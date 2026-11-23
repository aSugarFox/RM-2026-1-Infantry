#include "infantry_indicator.h"

#include "bsp_buzzer.h"
#include "infantry_config.h"
#include "main.h"

static uint8_t beep_pending;
static uint8_t beep_queued;
static uint8_t tone_on;
static uint8_t pattern_gap;
static uint32_t next_tick;

void Infantry_Indicator_Init(void)
{
    beep_pending = 0u;
    beep_queued = 0u;
    tone_on = 0u;
    pattern_gap = 0u;
    next_tick = HAL_GetTick();
    BEEP_SetFrequency(0u);
}

void Infantry_Indicator_RequestBeeps(uint8_t count)
{
    if ((INFANTRY_BUZZER_ENABLE == 0u) || (count == 0u)) return;

    if ((beep_pending == 0u) && (tone_on == 0u) && (pattern_gap == 0u))
    {
        beep_pending = count;
        next_tick = HAL_GetTick();
    }
    else if (beep_queued == 0u)
    {
        beep_queued = count;
    }
}

void Infantry_Indicator_Poll(void)
{
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - next_tick) < 0) return;

    if (pattern_gap != 0u)
    {
        pattern_gap = 0u;
        beep_pending = beep_queued;
        beep_queued = 0u;
    }

    if (tone_on != 0u)
    {
        BEEP_SetFrequency(0u);
        tone_on = 0u;
        if ((beep_pending == 0u) && (beep_queued != 0u))
        {
            pattern_gap = 1u;
            next_tick = now + INFANTRY_BUZZER_PATTERN_GAP_MS;
        }
        else
        {
            next_tick = now + INFANTRY_BUZZER_GAP_MS;
        }
    }
    else if (beep_pending != 0u)
    {
        beep_pending--;
        BEEP_SetFrequency(INFANTRY_BUZZER_FREQUENCY_HZ);
        tone_on = 1u;
        next_tick = now + INFANTRY_BUZZER_ON_MS;
    }
}
