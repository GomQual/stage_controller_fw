#ifndef INC_COMM_H_
#define INC_COMM_H_

#include "main.h"

#define COMM_TIMEOUT	(9999U)
#define MAX_BUFFER_SIZE	(1024U)

void COMM_IT_Receive(UART_HandleTypeDef *huart, uint8_t data);
void COMM_ButtonControl(UART_HandleTypeDef *huart, uint8_t data);
void COMM_G_CMD(UART_HandleTypeDef *huart, uint8_t data);
uint8_t COMM_CheckCS(char str[], int len);
void COMM_Print(UART_HandleTypeDef *huart, const char *str, ...);
char BYTE2HEX(uint8_t byte);
uint8_t HEX2BYTE(char c);

extern char comm_buf[MAX_BUFFER_SIZE];
extern int comm_bufIdxH;
extern int comm_bufIdxT;

#endif /* INC_COMM_H_ */
