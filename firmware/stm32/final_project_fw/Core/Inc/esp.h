#ifndef __ESP_H__
#define __ESP_H__

#include "stm32f4xx_hal.h"
#include <stddef.h>
#include <stdbool.h>

#define MAX_ESP_RX_BUFFER      1024
#define MAX_ESP_COMMAND_LEN    64
#define MAX_ESP_CLIENT_NUM     10

#define SSID "Cho"
#define PASS "cho135790"
#define LOGID "stm"
#define DST_IP "192.168.0.2"
#define DST_PORT 9190

typedef struct _cb_data_t
{
    uint8_t buf[MAX_ESP_RX_BUFFER];
    uint16_t length;
}cb_data_t;

int drv_esp_init(void);
int drv_esp_test_command(void);
void AiotClient_Init(void);
int esp_client_conn(void);
void esp_send_data(char *data);
int esp_get_status(void);
int esp_fetch_message(char *dst, size_t dst_len);
bool esp_has_pending(void);

//==================uart3=========================
#define MAX_UART_RX_BUFFER      512
#define MAX_UART_COMMAND_LEN    64

int drv_uart_init(void);
int drv_uart_rx_buffer(uint8_t *buf, uint16_t size);
int drv_uart_tx_buffer(uint8_t *buf, uint16_t size);
//==================uart3=========================

#endif
