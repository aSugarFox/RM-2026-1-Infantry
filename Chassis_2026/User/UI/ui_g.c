//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"
#include "main.h"

extern void print_message(const uint8_t *message, const int length);
ui_5_frame_t ui_g_Ungroup_0;

ui_interface_line_t *ui_g_Ungroup_line1 = (ui_interface_line_t*)&(ui_g_Ungroup_0.data[0]);
ui_interface_line_t *ui_g_Ungroup_line2 = (ui_interface_line_t*)&(ui_g_Ungroup_0.data[1]);
ui_interface_line_t *ui_g_Ungroup_line3 = (ui_interface_line_t*)&(ui_g_Ungroup_0.data[2]);
ui_interface_line_t *ui_g_Ungroup_line4 = (ui_interface_line_t*)&(ui_g_Ungroup_0.data[3]);

void _ui_init_g_Ungroup_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_Ungroup_0.data[i].figure_name[0] = 0;
        ui_g_Ungroup_0.data[i].figure_name[1] = 0;
        ui_g_Ungroup_0.data[i].figure_name[2] = i + 0;
        ui_g_Ungroup_0.data[i].operate_type = 1;
    }
    for (int i = 4; i < 5; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 0;
    }

    ui_g_Ungroup_line1->figure_type = 0;
    ui_g_Ungroup_line1->operate_type = 1;
    ui_g_Ungroup_line1->layer = 0;
    ui_g_Ungroup_line1->color = 0;
    ui_g_Ungroup_line1->start_x = 959;
    ui_g_Ungroup_line1->start_y = 350;
    ui_g_Ungroup_line1->width = 2;
    ui_g_Ungroup_line1->end_x = 959;
    ui_g_Ungroup_line1->end_y = 751;

    ui_g_Ungroup_line2->figure_type = 0;
    ui_g_Ungroup_line2->operate_type = 1;
    ui_g_Ungroup_line2->layer = 0;
    ui_g_Ungroup_line2->color = 0;
    ui_g_Ungroup_line2->start_x = 843;
    ui_g_Ungroup_line2->start_y = 561;
    ui_g_Ungroup_line2->width = 1;
    ui_g_Ungroup_line2->end_x = 1079;
    ui_g_Ungroup_line2->end_y = 563;

    ui_g_Ungroup_line3->figure_type = 0;
    ui_g_Ungroup_line3->operate_type = 1;
    ui_g_Ungroup_line3->layer = 0;
    ui_g_Ungroup_line3->color = 0;
    ui_g_Ungroup_line3->start_x = 828;
    ui_g_Ungroup_line3->start_y = 525;
    ui_g_Ungroup_line3->width = 1;
    ui_g_Ungroup_line3->end_x = 1093;
    ui_g_Ungroup_line3->end_y = 528;

    ui_g_Ungroup_line4->figure_type = 0;
    ui_g_Ungroup_line4->operate_type = 1;
    ui_g_Ungroup_line4->layer = 0;
    ui_g_Ungroup_line4->color = 0;
    ui_g_Ungroup_line4->start_x = 840;
    ui_g_Ungroup_line4->start_y = 496;
    ui_g_Ungroup_line4->width = 1;
    ui_g_Ungroup_line4->end_x = 1084;
    ui_g_Ungroup_line4->end_y = 499;


    ui_proc_5_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_update_g_Ungroup_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
    /* No synchronous hex dump on the referee UART inside the control loop. */
}

void _ui_remove_g_Ungroup_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}


void ui_init_g_Ungroup() {
    _ui_init_g_Ungroup_0();
}

void ui_update_g_Ungroup() {
    _ui_update_g_Ungroup_0();
}

void ui_remove_g_Ungroup() {
    _ui_remove_g_Ungroup_0();
}

#ifndef UI_AUTO_UPDATE_PERIOD_MS
#define UI_AUTO_UPDATE_PERIOD_MS 100u
#endif

static uint32_t ui_last_update_tick = 0u;

void ui_auto_init(void)
{
    ui_set_self_id(UI_CFG_SELF_ID);
    ui_init_g_Ungroup();
    ui_last_update_tick = HAL_GetTick();
}

void ui_auto_task(void)
{
    uint32_t now_tick = HAL_GetTick();

    if ((uint32_t)(now_tick - ui_last_update_tick) >= UI_AUTO_UPDATE_PERIOD_MS)
    {
        ui_last_update_tick = now_tick;
        ui_update_g_Ungroup();
    }
}
