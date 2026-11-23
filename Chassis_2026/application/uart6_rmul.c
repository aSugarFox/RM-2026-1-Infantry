#include "uart6_rmul.h"

static uart6_rmul_t uart6 = {0};
static uint8_t tx_buffer[UART6_TX_BUFFER_SIZE];

static const uint8_t CRC8_TAB[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
};

static const uint16_t CRC16_TAB[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

#define CRC8_INIT  0xff
#define CRC16_INIT 0xffff

static uint16_t uart6_read_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void uart6_write_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static void uart6_copy_with_zero(void *dst, uint16_t dst_len, const uint8_t *src, uint16_t src_len)
{
    uint16_t copy_len = src_len;
    if (copy_len > dst_len) {
        copy_len = dst_len;
    }
    memset(dst, 0, dst_len);
    if (copy_len > 0U) {
        memcpy(dst, src, copy_len);
    }
}

static void uart6_reset_parser(void)
{
    uart6.parse_state = STATE_WAIT_SOF;
    uart6.rx_index = 0;
    uart6.expected_len = 0;
}

static void uart6_start_receive_it(void)
{
    if (uart6.huart == NULL) {
        return;
    }

    if (HAL_UART_Receive_IT(uart6.huart, &uart6.rx_byte, 1U) == HAL_OK) {
        uart6.rx_rearm_count++;
    } else {
        uart6.error_count++;
    }
}

/* CRC8 */
uint8_t get_crc8_check_sum(uint8_t *pchMessage, uint32_t dwLength, uint8_t ucCRC8)
{
    uint8_t ucIndex;
    while (dwLength--) {
        ucIndex = ucCRC8 ^ (*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return ucCRC8;
}

uint32_t verify_crc8_check_sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint8_t ucExpected;
    if ((pchMessage == NULL) || (dwLength <= 2U)) {
        return 0U;
    }
    ucExpected = get_crc8_check_sum(pchMessage, dwLength - 1U, CRC8_INIT);
    return (ucExpected == pchMessage[dwLength - 1U]) ? 1U : 0U;
}

void append_crc8_check_sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint8_t ucCRC;
    if ((pchMessage == NULL) || (dwLength <= 2U)) {
        return;
    }
    ucCRC = get_crc8_check_sum(pchMessage, dwLength - 1U, CRC8_INIT);
    pchMessage[dwLength - 1U] = ucCRC;
}

/* CRC16 */
uint16_t get_crc16_check_sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == NULL) {
        return 0xFFFFU;
    }
    while (dwLength--) {
        chData = *pchMessage++;
        wCRC = (uint16_t)((wCRC >> 8) ^ CRC16_TAB[(wCRC ^ chData) & 0x00FFU]);
    }
    return wCRC;
}

uint32_t verify_crc16_check_sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected;
    if ((pchMessage == NULL) || (dwLength <= 2U)) {
        return 0U;
    }
    wExpected = get_crc16_check_sum(pchMessage, dwLength - 2U, CRC16_INIT);
    return ((uint8_t)(wExpected & 0xFFU) == pchMessage[dwLength - 2U] &&
            (uint8_t)((wExpected >> 8) & 0xFFU) == pchMessage[dwLength - 1U]) ? 1U : 0U;
}

void append_crc16_check_sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wCRC;
    if ((pchMessage == NULL) || (dwLength <= 2U)) {
        return;
    }
    wCRC = get_crc16_check_sum(pchMessage, dwLength - 2U, CRC16_INIT);
    pchMessage[dwLength - 2U] = (uint8_t)(wCRC & 0x00FFU);
    pchMessage[dwLength - 1U] = (uint8_t)((wCRC >> 8) & 0x00FFU);
}

static void uart6_dispatch_cmd(uint16_t cmd_id, const uint8_t *cmd_data, uint16_t cmd_len)
{
    switch (cmd_id) {
    case CMD_GAME_STATUS:
        if (cmd_len == sizeof(game_status_t)) {
            uart6_copy_with_zero(&uart6.game_status, sizeof(game_status_t), cmd_data, cmd_len);
            uart6.game_status_valid = 1U;
        }
        break;

    case CMD_ROBOT_HP:
        if (cmd_len == sizeof(robot_hp_t)) {
            uart6_copy_with_zero(&uart6.robot_hp, sizeof(robot_hp_t), cmd_data, cmd_len);
            uart6.robot_hp_valid = 1U;
        }
        break;

    case CMD_ROBOT_STATUS:
        if (cmd_len == sizeof(robot_performance_t)) {
            uart6_copy_with_zero(&uart6.robot_perf, sizeof(robot_performance_t), cmd_data, cmd_len);
            uart6.robot_perf_valid = 1U;
        }
        break;

    case CMD_POWER_HEAT_DATA:
        if (cmd_len == sizeof(power_heat_data_t)) {
            uart6_copy_with_zero(&uart6.power_heat, sizeof(power_heat_data_t), cmd_data, cmd_len);
            uart6.power_heat_valid = 1U;
        }
        break;

    case CMD_ROBOT_POS:
        if (cmd_len == sizeof(robot_pos_t)) {
            uart6_copy_with_zero(&uart6.robot_pos, sizeof(robot_pos_t), cmd_data, cmd_len);
            uart6.robot_pos_valid = 1U;
        }
        break;

    case CMD_BUFF_DATA:
        if (cmd_len == sizeof(buff_data_t)) {
            uart6_copy_with_zero(&uart6.buff_data, sizeof(buff_data_t), cmd_data, cmd_len);
            uart6.buff_data_valid = 1U;
        }
        break;

    case CMD_HURT_DATA:
        if (cmd_len == sizeof(hurt_data_t)) {
            uart6_copy_with_zero(&uart6.hurt_data, sizeof(hurt_data_t), cmd_data, cmd_len);
            uart6.hurt_data_valid = 1U;
        }
        break;

    case CMD_SHOOT_DATA:
        if (cmd_len == sizeof(shoot_data_t)) {
            uart6_copy_with_zero(&uart6.shoot_data, sizeof(shoot_data_t), cmd_data, cmd_len);
            uart6.shoot_data_valid = 1U;
        }
        break;

    case CMD_PROJECTILE_ALLOWANCE:
        if (cmd_len == 6U || cmd_len == sizeof(projectile_allowance_t)) {
            uart6_copy_with_zero(&uart6.projectile_allowance, sizeof(projectile_allowance_t), cmd_data, cmd_len);
            uart6.projectile_allowance_valid = 1U;
        }
        break;

    case CMD_RFID_STATUS:
        if (cmd_len == sizeof(rfid_status_t)) {
            uart6_copy_with_zero(&uart6.rfid_status, sizeof(rfid_status_t), cmd_data, cmd_len);
            uart6.rfid_status_valid = 1U;
        }
        break;

    case CMD_CUSTOM_CLIENT_DATA:
        if (cmd_len == sizeof(custom_client_data_t)) {
            uart6_copy_with_zero(&uart6.custom_data, sizeof(custom_client_data_t), cmd_data, cmd_len);
            uart6.custom_data_valid = 1U;
        }
        break;

    default:
        break;
    }
}

static void uart6_parse_frame(const uint8_t *frame, uint16_t frame_len)
{
    uint16_t data_len;
    uint16_t total_len;
    uint16_t cmd_id;

    if (frame_len < (FRAME_HEADER_SIZE + CMD_ID_SIZE + CRC16_SIZE)) {
        uart6.error_count++;
        return;
    }

    if (frame[0] != FRAME_HEADER_SOF) {
        uart6.error_count++;
        return;
    }

    if (!verify_crc8_check_sum((uint8_t *)frame, FRAME_HEADER_SIZE)) {
        uart6.error_count++;
        return;
    }

    data_len = uart6_read_u16_le(&frame[1]);
    total_len = (uint16_t)(FRAME_HEADER_SIZE + CMD_ID_SIZE + data_len + CRC16_SIZE);
    if (total_len != frame_len || total_len > UART6_RX_BUFFER_SIZE) {
        uart6.error_count++;
        return;
    }

    if (!verify_crc16_check_sum((uint8_t *)frame, total_len)) {
        uart6.error_count++;
        return;
    }

    cmd_id = uart6_read_u16_le(&frame[FRAME_HEADER_SIZE]);
    uart6.last_cmd_id = cmd_id;
    uart6.frame_count++;

    uart6_dispatch_cmd(cmd_id, &frame[FRAME_HEADER_SIZE + CMD_ID_SIZE], data_len);

    if (uart6.data_ready_callback != NULL) {
        uart6.data_ready_callback();
    }
}

static void uart6_parse_byte(uint8_t byte)
{
    uart6.rx_byte_count++;

    switch (uart6.parse_state) {
    case STATE_WAIT_SOF:
        if (byte == FRAME_HEADER_SOF) {
            uart6.rx_buffer[0] = byte;
            uart6.rx_index = 1;
            uart6.parse_state = STATE_RECV_HEADER;
        }
        break;

    case STATE_RECV_HEADER:
        uart6.rx_buffer[uart6.rx_index++] = byte;
        if (uart6.rx_index >= FRAME_HEADER_SIZE) {
            uint16_t data_len;
            if (!verify_crc8_check_sum(uart6.rx_buffer, FRAME_HEADER_SIZE)) {
                uart6.error_count++;
                uart6_reset_parser();
                break;
            }

            data_len = uart6_read_u16_le(&uart6.rx_buffer[1]);
            uart6.expected_len = (uint16_t)(FRAME_HEADER_SIZE + CMD_ID_SIZE + data_len + CRC16_SIZE);
            if (uart6.expected_len < (FRAME_HEADER_SIZE + CMD_ID_SIZE + CRC16_SIZE) ||
                uart6.expected_len > UART6_RX_BUFFER_SIZE) {
                uart6.error_count++;
                uart6_reset_parser();
                break;
            }
            uart6.parse_state = STATE_RECV_BODY;
        }
        break;

    case STATE_RECV_BODY:
        if (uart6.rx_index >= UART6_RX_BUFFER_SIZE) {
            uart6.error_count++;
            uart6_reset_parser();
            break;
        }

        uart6.rx_buffer[uart6.rx_index++] = byte;
        if (uart6.rx_index >= uart6.expected_len) {
            memcpy(uart6.temp_buffer, uart6.rx_buffer, uart6.expected_len);
            uart6_parse_frame(uart6.temp_buffer, uart6.expected_len);
            uart6_reset_parser();
        }
        break;

    default:
        uart6_reset_parser();
        break;
    }
}

void uart6_rmul_init(UART_HandleTypeDef *huart)
{
    memset(&uart6, 0, sizeof(uart6));
    uart6.huart = huart;
    uart6_reset_parser();
    uart6_start_receive_it();
}

void uart6_rmul_set_callback(void (*callback)(void))
{
    uart6.data_ready_callback = callback;
}

void uart6_rmul_irq_handler(void)
{
    if (uart6.huart != NULL) {
        HAL_UART_IRQHandler(uart6.huart);
    }
}

void uart6_rmul_poll(void)
{
    if (uart6.huart == NULL) {
        return;
    }

    /* Recover if receive state accidentally stops. */
    if (uart6.huart->RxState != HAL_UART_STATE_BUSY_RX) {
        __HAL_UART_CLEAR_OREFLAG(uart6.huart);
        uart6_reset_parser();
        uart6_start_receive_it();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((uart6.huart != NULL) && (huart == uart6.huart)) {
        uart6.rx_irq_count++;
        uart6_parse_byte(uart6.rx_byte);
        uart6_start_receive_it();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((uart6.huart != NULL) && (huart == uart6.huart)) {
        uart6.error_count++;
        __HAL_UART_CLEAR_PEFLAG(huart);
        uart6_reset_parser();
        uart6_start_receive_it();
    }
}

uint8_t uart6_rmul_send_data(uint16_t cmd_id, uint8_t *data, uint16_t len)
{
    uint16_t total_len;

    if (uart6.huart == NULL) {
        return 0U;
    }

    total_len = (uint16_t)(FRAME_HEADER_SIZE + CMD_ID_SIZE + len + CRC16_SIZE);
    if (total_len > UART6_TX_BUFFER_SIZE) {
        return 0U;
    }

    tx_buffer[0] = FRAME_HEADER_SOF;
    uart6_write_u16_le(&tx_buffer[1], len);
    tx_buffer[3] = uart6.tx_seq++;
    append_crc8_check_sum(tx_buffer, FRAME_HEADER_SIZE);

    uart6_write_u16_le(&tx_buffer[FRAME_HEADER_SIZE], cmd_id);

    if ((len > 0U) && (data != NULL)) {
        memcpy(&tx_buffer[FRAME_HEADER_SIZE + CMD_ID_SIZE], data, len);
    }

    append_crc16_check_sum(tx_buffer, total_len);

    return (HAL_UART_Transmit(uart6.huart, tx_buffer, total_len, 100U) == HAL_OK) ? 1U : 0U;
}

game_status_t* uart6_get_game_status(void)
{
    return &uart6.game_status;
}

robot_hp_t* uart6_get_robot_hp(void)
{
    return &uart6.robot_hp;
}

robot_performance_t* uart6_get_robot_perf(void)
{
    return &uart6.robot_perf;
}

power_heat_data_t* uart6_get_power_heat(void)
{
    return &uart6.power_heat;
}

robot_pos_t* uart6_get_robot_pos(void)
{
    return &uart6.robot_pos;
}

buff_data_t* uart6_get_buff_data(void)
{
    return &uart6.buff_data;
}

hurt_data_t* uart6_get_hurt_data(void)
{
    return &uart6.hurt_data;
}

shoot_data_t* uart6_get_shoot_data(void)
{
    return &uart6.shoot_data;
}

projectile_allowance_t* uart6_get_projectile_allowance(void)
{
    return &uart6.projectile_allowance;
}

rfid_status_t* uart6_get_rfid_status(void)
{
    return &uart6.rfid_status;
}

custom_client_data_t* uart6_get_custom_data(void)
{
    return &uart6.custom_data;
}

uint8_t uart6_is_game_status_updated(void)
{
    uint8_t valid = uart6.game_status_valid;
    uart6.game_status_valid = 0;
    return valid;
}

uint8_t uart6_is_robot_hp_updated(void)
{
    uint8_t valid = uart6.robot_hp_valid;
    uart6.robot_hp_valid = 0;
    return valid;
}

uint8_t uart6_is_robot_perf_updated(void)
{
    uint8_t valid = uart6.robot_perf_valid;
    uart6.robot_perf_valid = 0;
    return valid;
}

uint8_t uart6_is_power_heat_updated(void)
{
    uint8_t valid = uart6.power_heat_valid;
    uart6.power_heat_valid = 0;
    return valid;
}

uint8_t uart6_is_robot_pos_updated(void)
{
    uint8_t valid = uart6.robot_pos_valid;
    uart6.robot_pos_valid = 0;
    return valid;
}

uint32_t uart6_get_frame_count(void)
{
    return uart6.frame_count;
}

uint32_t uart6_get_error_count(void)
{
    return uart6.error_count;
}

uint16_t uart6_get_last_cmd_id(void)
{
    return uart6.last_cmd_id;
}

uint32_t uart6_get_rx_byte_count(void)
{
    return uart6.rx_byte_count;
}

uint32_t uart6_get_rx_irq_count(void)
{
    return uart6.rx_irq_count;
}

uint32_t uart6_get_rx_rearm_count(void)
{
    return uart6.rx_rearm_count;
}

uint32_t uart6_get_rx_state(void)
{
    if (uart6.huart == NULL) {
        return 0xFFFFFFFFU;
    }
    return (uint32_t)uart6.huart->RxState;
}
