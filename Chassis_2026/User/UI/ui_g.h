//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_line_t *ui_g_Ungroup_line1;
extern ui_interface_line_t *ui_g_Ungroup_line2;
extern ui_interface_line_t *ui_g_Ungroup_line3;
extern ui_interface_line_t *ui_g_Ungroup_line4;

void ui_init_g_Ungroup();
void ui_update_g_Ungroup();
void ui_remove_g_Ungroup();

void ui_auto_init(void);
void ui_auto_task(void);

#endif // UI_g_H
