/*
 BitzOS (BOS) V0.2.7 - Copyright (C) 2017-2022 Hexabitz
 All rights reserved
 
 File Name     : H3BR6_MemoryMap.h
 Description   : Module MCU memory map header file.

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H3BR6_MEMORYMAP_H
#define H3BR6_MEMORYMAP_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Memory map: - STM32G0B1CE
 - Application: 0x08000000 - 0x0807D7FF >> 502 KB
 - Read-only (RO): 0x0807D800 - 0x0807DFFF >> 2 KB, used to store topology information and Command Snippets
 - Emulated EEPROM: 0x0807E000 - 0x0807FFFF >>8 KB, fits 1024 16-bit variables in 2 main-duplicate pages (A and B)
 */
#ifdef FLASH_SIZE
	#undef FLASH_SIZE
#endif

#define APP_START_ADDRESS  		((uint32_t)0x08000000)
#define RO_START_ADDRESS  		((uint32_t)0x0807D800)
#define RO_MID_ADDRESS  		((uint32_t)0x0807D400) 		// Snippets are stored here
#define EEPROM_START_ADDRESS  	((uint32_t)0x0807E000)
#define FLASH_SIZE				((uint32_t)0x80000)	        //TOBECHECKED		// All sizes in bytes
#define SRAM_SIZE				((uint32_t)0x24000)         //size for G0=144kb
#define PAGE_SIZE               ((uint32_t)0x0800)  		/* Page size = 2KByte  */
#define NumOfPages				(FLASH_SIZE/PAGE_SIZE)

#ifdef __cplusplus
}
#endif

#endif /* H3BR6_MEMORYMAP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
