#ifndef __UART6_RMUL_H
#define __UART6_RMUL_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Protocol frame constants */
#define FRAME_HEADER_SOF         0xA5
#define FRAME_HEADER_SIZE        5
#define CMD_ID_SIZE              2
#define CRC16_SIZE               2

/* Buffer sizes */
#define UART6_RX_BUFFER_SIZE     256
#define UART6_TX_BUFFER_SIZE     256

/* Common RMUL cmd IDs on normal link */
#define CMD_GAME_STATUS          0x0001
#define CMD_GAME_RESULT          0x0002
#define CMD_ROBOT_HP             0x0003
#define CMD_ROBOT_STATUS         0x0201
#define CMD_ROBOT_PERFORMANCE    CMD_ROBOT_STATUS
#define CMD_POWER_HEAT_DATA      0x0202
#define CMD_ROBOT_POS            0x0203
#define CMD_BUFF_DATA            0x0204
#define CMD_HURT_DATA            0x0206
#define CMD_SHOOT_DATA           0x0207
#define CMD_PROJECTILE_ALLOWANCE 0x0208
#define CMD_RFID_STATUS          0x0209
#define CMD_CUSTOM_CLIENT_DATA   0x0306

/* frame_header */
typedef __packed struct {
    uint8_t sof;
    uint16_t data_length;
    uint8_t seq;
    uint8_t crc8;
} frame_header_t;

/* 0x0001 */
typedef __packed struct {
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t sync_time_stamp;
} game_status_t;

/* 0x0002 */
typedef __packed struct {
    uint8_t winner;
} game_result_t;

/* 0x0003 */
typedef __packed struct {
    uint16_t ally_1_robot_HP;
    uint16_t ally_2_robot_HP;
    uint16_t ally_3_robot_HP;
    uint16_t ally_4_robot_HP;
    uint16_t reserved;
    uint16_t ally_7_robot_HP;
    uint16_t ally_outpost_HP;
    uint16_t ally_base_HP;
} robot_hp_t;

/* 0x0201 */
typedef __packed struct {
    uint8_t robot_id;//机器人ID
    uint8_t robot_level;//机器人等级
    uint16_t current_HP;//当前HP
    uint16_t maximum_HP;//最大HP
    uint16_t shooter_barrel_cooling_value;//枪管冷却值
    uint16_t shooter_barrel_heat_limit;//枪管过热限制值
    uint16_t chassis_power_limit;//底盘功率限制值
    uint8_t power_management_output;//能量机关输出
} robot_performance_t;

/* 0x0202 */
typedef __packed struct {
    uint16_t reserved_1;//
    uint16_t reserved_2;    //
    float reserved_3;//保留
    uint16_t buffer_energy;//缓冲能量
    uint16_t shooter_17mm_heat;//17mm枪口热量
    uint16_t shooter_42mm_heat;
} power_heat_data_t;

/* 0x0203 */
typedef __packed struct {
    float x;
    float y;
    float angle;
} robot_pos_t;

/* 0x0204 */
typedef __packed struct {
    uint8_t recovery_buff;
    uint16_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy;
} buff_data_t;

/* 0x0206 */
typedef __packed struct {
    uint8_t armor_id : 4;
    uint8_t hp_deduction_reason : 4;
} hurt_data_t;

/* 0x0207 */
typedef __packed struct {
    uint8_t bullet_type;//弹丸类型
    uint8_t shooter_number;//发射器编号
    uint8_t launching_frequency;//发射频率
    float initial_speed;//初速
} shoot_data_t;

/* 0x0208 */
typedef __packed struct {
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
} projectile_allowance_t;

/* 0x0209 */
typedef __packed struct {
    uint32_t rfid_status;
    uint8_t rfid_status_2;
} rfid_status_t;

/* 0x0306 */
typedef __packed struct {
    uint16_t key_value;
    uint16_t x_position : 12;
    uint16_t mouse_left : 4;
    uint16_t y_position : 12;
    uint16_t mouse_right : 4;
    uint16_t reserved;
} custom_client_data_t;

/* keyboard bit mask */
#define KEY_W     (1U << 0)
#define KEY_S     (1U << 1)
#define KEY_A     (1U << 2)
#define KEY_D     (1U << 3)
#define KEY_SHIFT (1U << 4)
#define KEY_CTRL  (1U << 5)
#define KEY_Q     (1U << 6)
#define KEY_E     (1U << 7)
#define KEY_R     (1U << 8)
#define KEY_F     (1U << 9)
#define KEY_G     (1U << 10)
#define KEY_Z     (1U << 11)
#define KEY_X     (1U << 12)
#define KEY_C     (1U << 13)
#define KEY_V     (1U << 14)
#define KEY_B     (1U << 15)

typedef enum {
    STATE_WAIT_SOF = 0,
    STATE_RECV_HEADER,
    STATE_RECV_BODY
} uart_parse_state_t;

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t rx_byte;
    uint8_t rx_buffer[UART6_RX_BUFFER_SIZE];
    uint8_t temp_buffer[UART6_RX_BUFFER_SIZE];
    uint16_t rx_index;
    uint16_t expected_len;
    uint8_t tx_seq;
    uart_parse_state_t parse_state;
    uint32_t frame_count;
    uint32_t error_count;
    uint16_t last_cmd_id;
    uint32_t rx_byte_count;
    uint32_t rx_irq_count;
    uint32_t rx_rearm_count;

    game_status_t game_status;
    robot_hp_t robot_hp;
    robot_performance_t robot_perf;
    power_heat_data_t power_heat;
    robot_pos_t robot_pos;
    buff_data_t buff_data;
    hurt_data_t hurt_data;
    shoot_data_t shoot_data;
    projectile_allowance_t projectile_allowance;
    rfid_status_t rfid_status;
    custom_client_data_t custom_data;

    uint8_t game_status_valid;
    uint8_t robot_hp_valid;
    uint8_t robot_perf_valid;
    uint8_t power_heat_valid;
    uint8_t robot_pos_valid;
    uint8_t buff_data_valid;
    uint8_t hurt_data_valid;
    uint8_t shoot_data_valid;
    uint8_t projectile_allowance_valid;
    uint8_t rfid_status_valid;
    uint8_t custom_data_valid;

    void (*data_ready_callback)(void);
} uart6_rmul_t;

void uart6_rmul_init(UART_HandleTypeDef *huart);
void uart6_rmul_set_callback(void (*callback)(void));
void uart6_rmul_irq_handler(void);
void uart6_rmul_poll(void);
uint8_t uart6_rmul_send_data(uint16_t cmd_id, uint8_t *data, uint16_t len);

game_status_t* uart6_get_game_status(void);
robot_hp_t* uart6_get_robot_hp(void);
robot_performance_t* uart6_get_robot_perf(void);
power_heat_data_t* uart6_get_power_heat(void);
robot_pos_t* uart6_get_robot_pos(void);
buff_data_t* uart6_get_buff_data(void);
hurt_data_t* uart6_get_hurt_data(void);
shoot_data_t* uart6_get_shoot_data(void);
projectile_allowance_t* uart6_get_projectile_allowance(void);
rfid_status_t* uart6_get_rfid_status(void);
custom_client_data_t* uart6_get_custom_data(void);

uint8_t uart6_is_game_status_updated(void);
uint8_t uart6_is_robot_hp_updated(void);
uint8_t uart6_is_robot_perf_updated(void);
uint8_t uart6_is_power_heat_updated(void);
uint8_t uart6_is_robot_pos_updated(void);
uint32_t uart6_get_frame_count(void);
uint32_t uart6_get_error_count(void);
uint16_t uart6_get_last_cmd_id(void);
uint32_t uart6_get_rx_byte_count(void);
uint32_t uart6_get_rx_irq_count(void);
uint32_t uart6_get_rx_rearm_count(void);
uint32_t uart6_get_rx_state(void);

uint8_t get_crc8_check_sum(uint8_t *pchMessage, uint32_t dwLength, uint8_t ucCRC8);
uint32_t verify_crc8_check_sum(uint8_t *pchMessage, uint32_t dwLength);
void append_crc8_check_sum(uint8_t *pchMessage, uint32_t dwLength);
uint16_t get_crc16_check_sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t verify_crc16_check_sum(uint8_t *pchMessage, uint32_t dwLength);
void append_crc16_check_sum(uint8_t *pchMessage, uint32_t dwLength);

#endif /* __UART6_RMUL_H */
