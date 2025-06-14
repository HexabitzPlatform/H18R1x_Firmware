/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name  : H18R1_uart.h
 Description: UART interface declarations for H18R1.
 Functions: Mutex-protected read/write operations (polling/IT modes).
 Comm: Port direction messaging support.
*/

/* Define to prevent recursive inclusion ***********************************/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ****************************************************************/
#include "BOS.h"

/* Exported Functions ******************************************************/
extern HAL_StatusTypeDef readPxMutex(uint8_t port,char *buffer,uint16_t n,uint32_t mutexTimeout,uint32_t portTimeout);
extern HAL_StatusTypeDef writePxMutex(uint8_t port,char *buffer,uint16_t n,uint32_t mutexTimeout,uint32_t portTimeout);
extern HAL_StatusTypeDef readPxITMutex(uint8_t port,char *buffer,uint16_t n,uint32_t mutexTimeout);
extern HAL_StatusTypeDef writePxITMutex(uint8_t port,char *buffer,uint16_t n,uint32_t mutexTimeout);
extern BOS_Status ReadPortsDirMSG(uint8_t SourceModule);

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
