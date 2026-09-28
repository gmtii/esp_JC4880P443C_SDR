#ifndef UART_COMMANDS_H
#define UART_COMMANDS_H

#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UART_CMD_UART_NUM   UART_NUM_0
#define UART_CMD_BUF_SIZE   1024

/**
 * @brief Inicializa el puerto UART para recibir comandos.
 */
void uart_init(void);

/**
 * @brief Loop que escucha datos por UART y gestiona comandos.
 *
 * Debe ejecutarse dentro de una tarea FreeRTOS:
 * xTaskCreate(uart_command_loop, "uart_cmd", 4096, NULL, 5, NULL);
 */
void uart_command_loop(void *arg);

/**
 * @brief Función que recibe la línea completa procesada como comando.
 * 
 * @param cmd Cadena terminada en '\0' con el comando recibido.
 */
void uart_command_handler(char *cmd);

#ifdef __cplusplus
}
#endif

#endif // UART_COMMANDS_H