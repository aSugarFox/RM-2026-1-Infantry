/*
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2026-4-12       zxy             鼠标控制射击状态机实现
  * 主要后续优化热量检测目前的热量检测，有个思路是单发就打一发转一个弹丸角度，连发就打7发然后锁住冷却会
   ****************************(C) COPYRIGHT 2024****************************        
  */
#include "Control_Shoot_MouseSM.h"
#include "usart6_remote_control.h"

#include <math.h>
#include <string.h>
/*------------------------------------------------------------------------------
鼠标控制射击的函数实现，主要有摩擦轮和波弹电机，
摩擦轮主要设置速度调pid，拨弹主要是卡弹电机会堵转，有卡弹检查和热量保护机制，但是热量保护比赛出问题没排查出来
还有鼠标数据的处理，单点和长按的区分，单点就是按一下打一下，长按就是按着不放持续打，
单点和长按的区分是通过按下持续的时间来判断的，超过一定时间就认为是长按了
-------------------------------------------------------------------------------*/
/* Friction wheel and trigger behavior */
#define MOUSE_SHOOT_FRIC_SPEED_RPM            6000     //原来5000
#define MOUSE_SHOOT_STEP_DEG                  45.0f   /* PI/4 */
#define MOUSE_SHOOT_BACKOFF_STEP_DEG          45.0f   /* reverse one step when jammed */
#define MOUSE_SHOOT_TARGET_REACHED_DEG        1.5f

/* Timing (main loop is currently 10 ms) */
#define MOUSE_SHOOT_LONG_PRESS_TICKS          15u     /* about 150 ms */
#define MOUSE_SHOOT_FRIC_READY_TICKS          20u     /* about 200 ms */
#define MOUSE_SHOOT_JAM_DETECT_TICKS          12u     /* about 120 ms */

/* Jam detection */
#define MOUSE_SHOOT_JAM_SPEED_RPM             120.0f
#define MOUSE_SHOOT_JAM_ERR_DEG               3.0f

/* Position hold + safety limits */
#define MOUSE_SHOOT_MAX_SPEED_CMD_RPM         2400.0f
#define MOUSE_SHOOT_MAX_POS_ERR_DEG           80.0f

/* Heat protection (17mm) */
#define MOUSE_SHOOT_HEAT_PER_SHOT             10u      /* official 17mm heat cost per shot */
#define MOUSE_SHOOT_HEAT_GUARD                0u       /* extra guard over one-shot budget */
#define MOUSE_SHOOT_HEAT_COOLDOWN_TICKS       10u      /* fallback cooldown when cooling value is unavailable */
#define MOUSE_SHOOT_HEAT_BLOCK_EXTRA_TICKS    0u       /* keep aggressive recovery near the heat limit */
#define MOUSE_SHOOT_HEAT_BLOCK_MAX_TICKS      300u     /* cap at about 3 s */
#define MOUSE_SHOOT_UART1_HEAT_TIMEOUT_MS     200u     /* UART1 heat frame freshness window */

/* Encoder constants for 2006 trigger motor */
#define MOUSE_SHOOT_ECD_RANGE                 8192
#define MOUSE_SHOOT_ECD_HALF_RANGE            4096
#define MOUSE_SHOOT_REDUCTION_RATIO           36.0f
#define MOUSE_SHOOT_ECD_TO_OUTPUT_DEG         (360.0f / (MOUSE_SHOOT_ECD_RANGE * MOUSE_SHOOT_REDUCTION_RATIO))

static mouse_shoot_sm_t g_mouse_shoot_sm;
static pid_type_def g_mouse_shoot_pos_pid;
static uint8_t g_feedback_inited = 0u;

/*
 * UART1 heat cache interface (reserved for future chassis->gimbal serial packet parser):
 * - valid = 1 means latest heat/limit should prefer UART1 cache.
 * - valid = 0 means fallback to referee heat source.
 */
static volatile uint16_t g_mouse_shoot_uart1_heat_limit = 0u;
static volatile uint16_t g_mouse_shoot_uart1_heat = 0u;
static volatile uint8_t g_mouse_shoot_uart1_heat_valid = 0u;

static uint16_t shoot_heat_predict_effective(uint16_t heat_effective,
                                             uint16_t cooling_value,
                                             uint32_t sync_tick,
                                             uint32_t now_tick);

static fp32 shoot_clampf(fp32 value, fp32 min_v, fp32 max_v)
{
    if (value > max_v)
    {
        return max_v;
    }
    if (value < min_v)
    {
        return min_v;
    }
    return value;
}

static void shoot_set_fric(uint8_t on)
{
    if (on)
    {
        RC_Control_Fric(MOUSE_SHOOT_FRIC_SPEED_RPM, -MOUSE_SHOOT_FRIC_SPEED_RPM);
    }
    else
    {
        RC_Control_Fric(0, 0);
    }
}

static void shoot_update_feedback(const motor_measure_t *trigger_motor)
{
    int32_t delta_ecd;

    if (trigger_motor == NULL)
    {
        return;
    }

    if (g_feedback_inited == 0u)
    {
        g_mouse_shoot_sm.last_ecd = trigger_motor->ecd;
        g_mouse_shoot_sm.total_ecd = 0;
        g_mouse_shoot_sm.current_angle_deg = 0.0f;
        g_mouse_shoot_sm.target_angle_deg = 0.0f;
        g_feedback_inited = 1u;
        return;
    }

    delta_ecd = (int32_t)trigger_motor->ecd - (int32_t)g_mouse_shoot_sm.last_ecd;
    if (delta_ecd > MOUSE_SHOOT_ECD_HALF_RANGE)
    {
        delta_ecd -= MOUSE_SHOOT_ECD_RANGE;
    }
    else if (delta_ecd < -MOUSE_SHOOT_ECD_HALF_RANGE)
    {
        delta_ecd += MOUSE_SHOOT_ECD_RANGE;
    }

    g_mouse_shoot_sm.total_ecd += delta_ecd;
    g_mouse_shoot_sm.last_ecd = trigger_motor->ecd;
    g_mouse_shoot_sm.current_angle_deg = (fp32)g_mouse_shoot_sm.total_ecd * MOUSE_SHOOT_ECD_TO_OUTPUT_DEG;
}

static uint8_t shoot_target_reached(fp32 error_deg)
{
    if (fabsf(error_deg) <= MOUSE_SHOOT_TARGET_REACHED_DEG)
    {
        return 1u;
    }
    return 0u;
}

static void shoot_limit_target(void)
{
    fp32 err = g_mouse_shoot_sm.target_angle_deg - g_mouse_shoot_sm.current_angle_deg;

    if (err > MOUSE_SHOOT_MAX_POS_ERR_DEG)
    {
        g_mouse_shoot_sm.target_angle_deg = g_mouse_shoot_sm.current_angle_deg + MOUSE_SHOOT_MAX_POS_ERR_DEG;
    }
    else if (err < -MOUSE_SHOOT_MAX_POS_ERR_DEG)
    {
        g_mouse_shoot_sm.target_angle_deg = g_mouse_shoot_sm.current_angle_deg - MOUSE_SHOOT_MAX_POS_ERR_DEG;
    }
}

static uint8_t shoot_jam_detect(const motor_measure_t *trigger_motor, fp32 pos_error_deg)
{
    uint8_t jam_cond;

    if (trigger_motor == NULL)
    {
        g_mouse_shoot_sm.jam_ticks = 0u;
        return 0u;
    }

    jam_cond = (uint8_t)((fabsf(trigger_motor->speed_rpm) <= MOUSE_SHOOT_JAM_SPEED_RPM) &&
                         (fabsf(pos_error_deg) >= MOUSE_SHOOT_JAM_ERR_DEG));

    if (jam_cond)
    {
        if (g_mouse_shoot_sm.jam_ticks < 0xFFFFu)
        {
            g_mouse_shoot_sm.jam_ticks++;
        }
    }
    else
    {
        g_mouse_shoot_sm.jam_ticks = 0u;
    }

    if (g_mouse_shoot_sm.jam_ticks >= MOUSE_SHOOT_JAM_DETECT_TICKS)
    {
        return 1u;
    }
    return 0u;
}

void Mouse_Shoot_SM_SetHeatFromUart1(uint16_t heat_limit, uint16_t heat, uint8_t valid)
{
    /*
     * This is a reserved interface for UART1 packet parser:
     * - call this function after parsing chassis->gimbal packet.
     * - until UART1 parser is ready, the state machine falls back to referee heat.
     */
    g_mouse_shoot_uart1_heat_limit = heat_limit;
    g_mouse_shoot_uart1_heat = heat;
    g_mouse_shoot_uart1_heat_valid = (uint8_t)((valid != 0u) ? 1u : 0u);
}

static void shoot_get_heat_limit_and_value(uint16_t *heat_limit,
                                           uint16_t *heat_now,
                                           uint16_t *cooling_value)
{
    uint16_t limit_tmp = 0u;
    uint16_t heat_tmp = 0u;
    uint16_t cooling_tmp = 0u;
    uint32_t now_tick;
    uint32_t uart1_tick;
    uint32_t uart1_age_ms;

    if ((heat_limit == NULL) || (heat_now == NULL) || (cooling_value == NULL))
    {
        return;
    }

    if (g_mouse_shoot_uart1_heat_valid != 0u)
    {
        /*
         * Highest priority: manually injected UART1 heat cache.
         * 该入口保留给后续串口解析模块直接喂热量数据。
        */
        limit_tmp = g_mouse_shoot_uart1_heat_limit;
        heat_tmp = g_mouse_shoot_uart1_heat;
        cooling_tmp = g_mouse_shoot_sm.heat_cooling_value;
    }
    else
    {
        now_tick = HAL_GetTick();
        uart1_tick = get_usart1_chassis_status_update_tick();

        if ((uart1_tick != 0u) && (now_tick >= uart1_tick))
        {
            uart1_age_ms = now_tick - uart1_tick;
        }
        else
        {
            uart1_age_ms = 0xFFFFFFFFu;
        }

        if (uart1_age_ms <= MOUSE_SHOOT_UART1_HEAT_TIMEOUT_MS)
        {
            /*
             * Preferred runtime source: chassis UART1 status frame.
             * 只要底盘状态帧足够新鲜，就优先使用串口热量。
            */
            limit_tmp = get_usart1_shooter_17mm_heat_limit();
            heat_tmp = get_usart1_shooter_17mm_heat();
            cooling_tmp = get_usart1_shooter_barrel_cooling_value();
        }
        else
        {
            /*
             * Fallback source (no fresh UART1 frame):
             * 1) 优先复用状态机里最近一次有效热量，避免因瞬时丢帧导致热量判断跳变。
             * 2) 如果历史值也没有（上电初期），则使用 0/0 作为安全默认值。
             *
             * 说明：
             * - 这里不再强依赖 referee 的 get_shoot_heat0_limit_and_heat0()，
             *   避免该符号在当前目标未链接时造成 L6218E 链接错误。
             * - 当后续底盘串口热量链路稳定后，仍建议保持 UART1 作为主热量来源。
             */
            if ((g_mouse_shoot_sm.heat_limit != 0u) || (g_mouse_shoot_sm.heat_now != 0u))
            {
                limit_tmp = g_mouse_shoot_sm.heat_limit;
                heat_tmp = g_mouse_shoot_sm.heat_now;
                cooling_tmp = g_mouse_shoot_sm.heat_cooling_value;
            }
            else
            {
                limit_tmp = 0u;
                heat_tmp = 0u;
                cooling_tmp = 0u;
            }
        }
    }

    *heat_limit = limit_tmp;
    *heat_now = heat_tmp;
    *cooling_value = cooling_tmp;
}

static void shoot_heat_sync_state(uint16_t heat_limit,
                                  uint16_t heat_now,
                                  uint16_t cooling_value)
{
    uint8_t heat_sample_changed = 0u;
    uint8_t meta_changed = 0u;
    uint32_t now_tick = HAL_GetTick();
    uint16_t predicted_heat = 0u;

    if ((g_mouse_shoot_sm.heat_data_valid == 0u) ||
        (heat_now != g_mouse_shoot_sm.heat_last_now))
    {
        heat_sample_changed = 1u;
    }
    else if ((heat_limit != g_mouse_shoot_sm.heat_limit) ||
             (cooling_value != g_mouse_shoot_sm.heat_cooling_value))
    {
        meta_changed = 1u;
    }

    if (meta_changed != 0u)
    {
        /*
         * 仅热量上限/冷却值变化时，先把旧冷却参数下的本地预测热量结算到当前时刻，
         * 再切到新的参数，避免把尚未回传的“保留一发热量”提前清掉。
         */
        predicted_heat = shoot_heat_predict_effective(g_mouse_shoot_sm.heat_effective,
                                                      g_mouse_shoot_sm.heat_cooling_value,
                                                      g_mouse_shoot_sm.heat_sync_tick,
                                                      now_tick);
        g_mouse_shoot_sm.heat_effective = predicted_heat;
        g_mouse_shoot_sm.heat_predicted = predicted_heat;
        g_mouse_shoot_sm.heat_sync_tick = now_tick;
    }

    g_mouse_shoot_sm.heat_limit = heat_limit;
    g_mouse_shoot_sm.heat_now = heat_now;
    g_mouse_shoot_sm.heat_last_now = heat_now;
    g_mouse_shoot_sm.heat_cooling_value = cooling_value;
    g_mouse_shoot_sm.heat_data_valid = (uint8_t)(((heat_limit != 0u) || (heat_now != 0u) || (cooling_value != 0u)) ? 1u : 0u);

    if (heat_sample_changed != 0u)
    {
        g_mouse_shoot_sm.heat_reserved = 0u;
        g_mouse_shoot_sm.heat_effective = heat_now;
        g_mouse_shoot_sm.heat_predicted = heat_now;
        g_mouse_shoot_sm.heat_sync_tick = now_tick;
    }
    else
    {
        g_mouse_shoot_sm.heat_predicted = shoot_heat_predict_effective(g_mouse_shoot_sm.heat_effective,
                                                                       g_mouse_shoot_sm.heat_cooling_value,
                                                                       g_mouse_shoot_sm.heat_sync_tick,
                                                                       now_tick);
    }
}

static uint16_t shoot_heat_predict_effective(uint16_t heat_effective,
                                             uint16_t cooling_value,
                                             uint32_t sync_tick,
                                             uint32_t now_tick)
{
    uint32_t elapsed_ms;
    uint32_t cool_steps_100ms;
    uint32_t cool_amount;

    if ((cooling_value == 0u) || (sync_tick == 0u) || (now_tick <= sync_tick))
    {
        return heat_effective;
    }

    elapsed_ms = now_tick - sync_tick;
    cool_steps_100ms = elapsed_ms / 100u;
    cool_amount = (cool_steps_100ms * (uint32_t)cooling_value) / 10u;

    if (cool_amount >= (uint32_t)heat_effective)
    {
        return 0u;
    }

    return (uint16_t)((uint32_t)heat_effective - cool_amount);
}

static uint16_t shoot_heat_calc_block_ticks(uint16_t heat_limit,
                                            uint16_t heat_effective,
                                            uint16_t cooling_value)
{
    uint16_t safe_heat;
    uint32_t required_cool;
    uint32_t cool_steps_100ms;
    uint32_t block_ticks;
    uint16_t predicted_heat;

    if (heat_limit == 0u)
    {
        return 0u;
    }

    predicted_heat = shoot_heat_predict_effective(heat_effective,
                                                  cooling_value,
                                                  g_mouse_shoot_sm.heat_sync_tick,
                                                  HAL_GetTick());
    g_mouse_shoot_sm.heat_predicted = predicted_heat;

    if (heat_limit > (MOUSE_SHOOT_HEAT_PER_SHOT + MOUSE_SHOOT_HEAT_GUARD))
    {
        safe_heat = (uint16_t)(heat_limit - MOUSE_SHOOT_HEAT_PER_SHOT - MOUSE_SHOOT_HEAT_GUARD);
    }
    else
    {
        safe_heat = 0u;
    }

    if (predicted_heat <= safe_heat)
    {
        return 0u;
    }

    required_cool = (uint32_t)predicted_heat - (uint32_t)safe_heat;

    if (cooling_value == 0u)
    {
        return MOUSE_SHOOT_HEAT_COOLDOWN_TICKS;
    }

    cool_steps_100ms = ((required_cool * 10u) + (uint32_t)cooling_value - 1u) / (uint32_t)cooling_value;
    block_ticks = cool_steps_100ms * 10u + MOUSE_SHOOT_HEAT_BLOCK_EXTRA_TICKS;

    if (block_ticks > MOUSE_SHOOT_HEAT_BLOCK_MAX_TICKS)
    {
        block_ticks = MOUSE_SHOOT_HEAT_BLOCK_MAX_TICKS;
    }

    return (uint16_t)block_ticks;
}

static uint8_t shoot_heat_can_start_shot(void)
{
    return (uint8_t)(shoot_heat_calc_block_ticks(g_mouse_shoot_sm.heat_limit,
                                                 g_mouse_shoot_sm.heat_effective,
                                                 g_mouse_shoot_sm.heat_cooling_value) == 0u);
}

static void shoot_heat_reserve_one_shot(void)
{
    uint32_t reserved_heat;
    uint32_t effective_heat;
    uint16_t predicted_heat;
    uint32_t now_tick;

    now_tick = HAL_GetTick();
    predicted_heat = shoot_heat_predict_effective(g_mouse_shoot_sm.heat_effective,
                                                  g_mouse_shoot_sm.heat_cooling_value,
                                                  g_mouse_shoot_sm.heat_sync_tick,
                                                  now_tick);
    g_mouse_shoot_sm.heat_effective = predicted_heat;

    reserved_heat = (uint32_t)g_mouse_shoot_sm.heat_reserved + MOUSE_SHOOT_HEAT_PER_SHOT;
    if (reserved_heat > 0xFFFFu)
    {
        reserved_heat = 0xFFFFu;
    }
    g_mouse_shoot_sm.heat_reserved = (uint16_t)reserved_heat;

    effective_heat = (uint32_t)predicted_heat + MOUSE_SHOOT_HEAT_PER_SHOT;
    if (effective_heat > 0xFFFFu)
    {
        effective_heat = 0xFFFFu;
    }
    g_mouse_shoot_sm.heat_effective = (uint16_t)effective_heat;
    g_mouse_shoot_sm.heat_predicted = (uint16_t)effective_heat;
    g_mouse_shoot_sm.heat_sync_tick = now_tick;
}
void Mouse_Shoot_SM_Init(void)
{
    static const fp32 pos_pid_param[3] = {130.0f, 0.0f, 12.0f};

    memset(&g_mouse_shoot_sm, 0, sizeof(g_mouse_shoot_sm));
    g_mouse_shoot_sm.mode = MOUSE_SHOOT_SM_DISABLED;
    g_mouse_shoot_sm.heat_block_ticks = 0u;
    g_mouse_shoot_sm.heat_limit = 0u;
    g_mouse_shoot_sm.heat_now = 0u;
    g_mouse_shoot_sm.heat_effective = 0u;
    g_mouse_shoot_sm.heat_reserved = 0u;
    g_mouse_shoot_sm.heat_last_now = 0u;
    g_mouse_shoot_sm.heat_cooling_value = 0u;
    g_mouse_shoot_sm.heat_predicted = 0u;
    g_mouse_shoot_sm.heat_blocked = 0u;
    g_mouse_shoot_sm.heat_data_valid = 0u;
    g_mouse_shoot_sm.heat_sync_tick = 0u;

    PID_init(&g_mouse_shoot_pos_pid,
             PID_POSITION,
             pos_pid_param,
             MOUSE_SHOOT_MAX_SPEED_CMD_RPM,
             700.0f);
    g_feedback_inited = 0u;
}

void Mouse_Shoot_SM_Reset(void)
{
    g_mouse_shoot_sm.mode = MOUSE_SHOOT_SM_DISABLED;
    g_mouse_shoot_sm.heat_block_ticks = 0u;
    g_mouse_shoot_sm.heat_limit = 0u;
    g_mouse_shoot_sm.heat_now = 0u;
    g_mouse_shoot_sm.heat_effective = 0u;
    g_mouse_shoot_sm.heat_reserved = 0u;
    g_mouse_shoot_sm.heat_last_now = 0u;
    g_mouse_shoot_sm.heat_cooling_value = 0u;
    g_mouse_shoot_sm.heat_predicted = 0u;
    g_mouse_shoot_sm.heat_blocked = 0u;
    g_mouse_shoot_sm.heat_data_valid = 0u;
    g_mouse_shoot_sm.heat_sync_tick = 0u;
    g_mouse_shoot_sm.fric_ready = 0u;
    g_mouse_shoot_sm.jammed = 0u;
    g_mouse_shoot_sm.left_hold_ticks = 0u;
    g_mouse_shoot_sm.fric_ready_ticks = 0u;
    g_mouse_shoot_sm.jam_ticks = 0u;
    g_mouse_shoot_sm.speed_set_rpm = 0.0f;
    g_mouse_shoot_sm.target_angle_deg = g_mouse_shoot_sm.current_angle_deg;

    PID_clear(&g_mouse_shoot_pos_pid);
    shoot_set_fric(0u);
    RC_Control_Bodan(0);
}

void Mouse_Shoot_SM_ForceNoFeed(void)
{
    /* Keep friction wheels running; only force feeder(2006) to stop. */
    g_mouse_shoot_sm.mode = MOUSE_SHOOT_SM_READY;
    g_mouse_shoot_sm.left_pressed = 0u;
    g_mouse_shoot_sm.left_pressed_last = 0u;
    g_mouse_shoot_sm.left_hold_ticks = 0u;
    g_mouse_shoot_sm.jam_ticks = 0u;
    g_mouse_shoot_sm.jammed = 0u;
    g_mouse_shoot_sm.target_angle_deg = g_mouse_shoot_sm.current_angle_deg;
    g_mouse_shoot_sm.speed_set_rpm = 0.0f;
    PID_clear(&g_mouse_shoot_pos_pid);
    RC_Control_Bodan(0);
}

/*
    @brief Mouse shoot state machine update
    @param single_shot_trigger level signal, pass left-button pressed level
*/
void Mouse_Shoot_SM_Update(uint8_t single_shot_trigger)
{
    const motor_measure_t *trigger_motor = get_trigger_motor_measure_point();
    uint8_t left_pressed_edge;
    uint8_t left_release_edge;
    uint16_t heat_limit = 0u;
    uint16_t heat_now = 0u;
    uint16_t heat_cooling_value = 0u;
    uint16_t heat_block_ticks_needed = 0u;
    fp32 pos_error_deg;

    /* friction wheels always on */
    shoot_set_fric(1u);

    if (trigger_motor == NULL)
    {
        RC_Control_Bodan(0);
        return;
    }

    shoot_update_feedback(trigger_motor);

    g_mouse_shoot_sm.left_pressed = (uint8_t)(single_shot_trigger != 0u);
    left_pressed_edge = (uint8_t)((g_mouse_shoot_sm.left_pressed != 0u) &&
                                  (g_mouse_shoot_sm.left_pressed_last == 0u));
    left_release_edge = (uint8_t)((g_mouse_shoot_sm.left_pressed == 0u) &&
                                  (g_mouse_shoot_sm.left_pressed_last != 0u));
    g_mouse_shoot_sm.left_pressed_last = g_mouse_shoot_sm.left_pressed;

    if (g_mouse_shoot_sm.left_pressed)
    {
        if (g_mouse_shoot_sm.left_hold_ticks < 0xFFFFu)
        {
            g_mouse_shoot_sm.left_hold_ticks++;
        }
    }
    else
    {
        g_mouse_shoot_sm.left_hold_ticks = 0u;
    }

    if (g_mouse_shoot_sm.fric_ready == 0u)
    {
        if (g_mouse_shoot_sm.fric_ready_ticks < 0xFFFFu)
        {
            g_mouse_shoot_sm.fric_ready_ticks++;
        }
        if (g_mouse_shoot_sm.fric_ready_ticks >= MOUSE_SHOOT_FRIC_READY_TICKS)
        {
            g_mouse_shoot_sm.fric_ready = 1u;
            g_mouse_shoot_sm.mode = MOUSE_SHOOT_SM_READY;
            g_mouse_shoot_sm.target_angle_deg = g_mouse_shoot_sm.current_angle_deg;
            PID_clear(&g_mouse_shoot_pos_pid);
        }
    }

    shoot_get_heat_limit_and_value(&heat_limit, &heat_now, &heat_cooling_value);
    shoot_heat_sync_state(heat_limit, heat_now, heat_cooling_value);
    heat_block_ticks_needed = shoot_heat_calc_block_ticks(g_mouse_shoot_sm.heat_limit,
                                                          g_mouse_shoot_sm.heat_effective,
                                                          g_mouse_shoot_sm.heat_cooling_value);

    /*
     * Thermal protection strategy:
     * 1) Block feeder when current heat is within one-shot budget of the limit.
     * 2) Keep a short hold-off after leaving the threshold to smooth telemetry jitter.
     * 3) Preserve long-press accumulation so continuous fire can resume naturally after cooling.
     */
    if (heat_block_ticks_needed > 0u)
    {
        g_mouse_shoot_sm.heat_block_ticks = heat_block_ticks_needed;
    }
    else if (g_mouse_shoot_sm.heat_block_ticks > 0u)
    {
        g_mouse_shoot_sm.heat_block_ticks--;
    }

    g_mouse_shoot_sm.heat_blocked = (uint8_t)((g_mouse_shoot_sm.heat_block_ticks > 0u) ? 1u : 0u);
    if (g_mouse_shoot_sm.heat_blocked != 0u)
    {
        g_mouse_shoot_sm.mode = MOUSE_SHOOT_SM_READY;
        g_mouse_shoot_sm.target_angle_deg = g_mouse_shoot_sm.current_angle_deg;
        g_mouse_shoot_sm.jam_ticks = 0u;
        g_mouse_shoot_sm.jammed = 0u;
        g_mouse_shoot_sm.speed_set_rpm = 0.0f;
        PID_clear(&g_mouse_shoot_pos_pid);
        RC_Control_Bodan(0);
        return;
    }
    pos_error_deg = g_mouse_shoot_sm.target_angle_deg - g_mouse_shoot_sm.current_angle_deg;

    switch (g_mouse_shoot_sm.mode)
    {
    case MOUSE_SHOOT_SM_DISABLED:
    {
        if (g_mouse_shoot_sm.fric_ready != 0u)
        {
            g_mouse_shoot_sm.mode = MOUSE_SHOOT_SM_READY;
            g_mouse_shoot_sm.target_angle_deg = g_mouse_shoot_sm.current_angle_deg;
        }
        break;
    }

    case MOUSE_SHOOT_SM_READY:
    {
        if (g_mouse_shoot_sm.fric_ready == 0u)
        {
            break;
        }

        if (left_pressed_edge != 0u)
        {
            if (shoot_heat_can_start_shot() != 0u)
            {
                g_mouse_shoot_sm.mode = MOUSE_SHOOT_SM_SINGLE;
                g_mouse_shoot_sm.target_angle_deg += MOUSE_SHOOT_STEP_DEG;
                g_mouse_shoot_sm.jam_ticks = 0u;
            }
        }
        else if ((g_mouse_shoot_sm.left_pressed != 0u) &&
                 (g_mouse_shoot_sm.left_hold_ticks >= MOUSE_SHOOT_LONG_PRESS_TICKS))
        {
            if (shoot_heat_can_start_shot() != 0u)
            {
                g_mouse_shoot_sm.mode = MOUSE_SHOOT_SM_CONTINUOUS;
                g_mouse_shoot_sm.target_angle_deg += MOUSE_SHOOT_STEP_DEG;
                g_mouse_shoot_sm.jam_ticks = 0u;
            }
        }
        break;
    }

    case MOUSE_SHOOT_SM_SINGLE:
    {
        if (shoot_jam_detect(trigger_motor, pos_error_deg) != 0u)
        {
            g_mouse_shoot_sm.mode = MOUSE_SHOOT_SM_JAM_BACKOFF;
            g_mouse_shoot_sm.jammed = 1u;
            g_mouse_shoot_sm.jam_ticks = 0u;
            g_mouse_shoot_sm.target_angle_deg = g_mouse_shoot_sm.current_angle_deg - MOUSE_SHOOT_BACKOFF_STEP_DEG;
            break;
        }

        if (shoot_target_reached(pos_error_deg) != 0u)
        {
            g_mouse_shoot_sm.single_shot_count++;
            shoot_heat_reserve_one_shot();
            g_mouse_shoot_sm.target_angle_deg = g_mouse_shoot_sm.current_angle_deg;

            if ((g_mouse_shoot_sm.left_pressed != 0u) &&
                (g_mouse_shoot_sm.left_hold_ticks >= MOUSE_SHOOT_LONG_PRESS_TICKS) &&
                (shoot_heat_can_start_shot() != 0u))
            {
                g_mouse_shoot_sm.mode = MOUSE_SHOOT_SM_CONTINUOUS;
                g_mouse_shoot_sm.target_angle_deg += MOUSE_SHOOT_STEP_DEG;
                g_mouse_shoot_sm.jam_ticks = 0u;
            }
            else
            {
                g_mouse_shoot_sm.mode = MOUSE_SHOOT_SM_READY;
                PID_clear(&g_mouse_shoot_pos_pid);
            }
        }
        break;
    }

    case MOUSE_SHOOT_SM_CONTINUOUS:
    {
        if (left_release_edge != 0u)
        {
            g_mouse_shoot_sm.mode = MOUSE_SHOOT_SM_READY;
            g_mouse_shoot_sm.target_angle_deg = g_mouse_shoot_sm.current_angle_deg;
            g_mouse_shoot_sm.jam_ticks = 0u;
            PID_clear(&g_mouse_shoot_pos_pid);
            break;
        }

        if (shoot_jam_detect(trigger_motor, pos_error_deg) != 0u)
        {
            g_mouse_shoot_sm.mode = MOUSE_SHOOT_SM_JAM_BACKOFF;
            g_mouse_shoot_sm.jammed = 1u;
            g_mouse_shoot_sm.jam_ticks = 0u;
            g_mouse_shoot_sm.target_angle_deg = g_mouse_shoot_sm.current_angle_deg - MOUSE_SHOOT_BACKOFF_STEP_DEG;
            break;
        }

        if (shoot_target_reached(pos_error_deg) != 0u)
        {
            g_mouse_shoot_sm.continuous_shot_count++;
            shoot_heat_reserve_one_shot();

            if (shoot_heat_can_start_shot() != 0u)
            {
                g_mouse_shoot_sm.target_angle_deg += MOUSE_SHOOT_STEP_DEG;
            }
            else
            {
                g_mouse_shoot_sm.mode = MOUSE_SHOOT_SM_READY;
                g_mouse_shoot_sm.target_angle_deg = g_mouse_shoot_sm.current_angle_deg;
                PID_clear(&g_mouse_shoot_pos_pid);
            }
        }
        break;
    }

    case MOUSE_SHOOT_SM_JAM_BACKOFF:
    {
        if (shoot_target_reached(pos_error_deg) != 0u)
        {
            g_mouse_shoot_sm.jammed = 0u;
            g_mouse_shoot_sm.jam_ticks = 0u;

            if ((g_mouse_shoot_sm.left_pressed != 0u) &&
                (g_mouse_shoot_sm.left_hold_ticks >= MOUSE_SHOOT_LONG_PRESS_TICKS) &&
                (shoot_heat_can_start_shot() != 0u))
            {
                g_mouse_shoot_sm.mode = MOUSE_SHOOT_SM_CONTINUOUS;
                g_mouse_shoot_sm.target_angle_deg = g_mouse_shoot_sm.current_angle_deg + MOUSE_SHOOT_STEP_DEG;
            }
            else
            {
                g_mouse_shoot_sm.mode = MOUSE_SHOOT_SM_READY;
                g_mouse_shoot_sm.target_angle_deg = g_mouse_shoot_sm.current_angle_deg;
                PID_clear(&g_mouse_shoot_pos_pid);
            }
        }
        break;
    }

    default:
    {
        g_mouse_shoot_sm.mode = MOUSE_SHOOT_SM_READY;
        g_mouse_shoot_sm.target_angle_deg = g_mouse_shoot_sm.current_angle_deg;
        g_mouse_shoot_sm.jam_ticks = 0u;
        g_mouse_shoot_sm.jammed = 0u;
        break;
    }
    }

    shoot_limit_target();
    g_mouse_shoot_sm.speed_set_rpm = PID_calc(&g_mouse_shoot_pos_pid,
                                              g_mouse_shoot_sm.current_angle_deg,
                                              g_mouse_shoot_sm.target_angle_deg);
    g_mouse_shoot_sm.speed_set_rpm = shoot_clampf(g_mouse_shoot_sm.speed_set_rpm,
                                                  -MOUSE_SHOOT_MAX_SPEED_CMD_RPM,
                                                  MOUSE_SHOOT_MAX_SPEED_CMD_RPM);
    RC_Control_Bodan((int16_t)g_mouse_shoot_sm.speed_set_rpm);
}

const mouse_shoot_sm_t *Mouse_Shoot_SM_Get(void)
{
    return &g_mouse_shoot_sm;
}

















