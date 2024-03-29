/**
 ******************************************************************************
 * @file    Src/command.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    4-April-2016
 * @brief   This file provides all the IAP command functions.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/** @addtogroup STM32L4xx_IAP_Main
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include "command.h"
#include "main.h"
#include "flash_if.h"
#include "fatfs.h"
//#include "memory_card.h"

/** @addtogroup USBH_USER
 * @{
 */

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define BUFFERSIZE     ((uint16_t)1*8192)  
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint8_t RAMBuf[BUFFERSIZE] = { 0xFF };

static uint32_t LastPGAddress = APPLICATION_ADDRESS;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief IAP Read all flash memory
 * @param  None
 * @retval None
 */
/*
void COMMAND_UPLOAD(void) {
	__IO uint32_t address = FLASH_BASE;
	__IO uint32_t counter_read = 0x00;
	uint32_t tmp_counter = 0x00, index_offset = 0x00;

	// Init written byte counter
	index_offset = 0x00;

	// Read flash memory
	while (index_offset != FLASH_SIZE) {
		for (counter_read = 0; counter_read < BUFFERSIZE; counter_read++) {
			// Check the read bytes versus the end of flash
			if (index_offset + counter_read != FLASH_SIZE) {
				tmp_counter = counter_read;
				RAMBuf[tmp_counter] = (*(uint8_t*) (address++));
			}
			// In this case all flash was read
			else {
				break;
			}
		}

		// Write buffer to file
		f_write(&MyFile, RAMBuf, counter_read, (UINT*) &tmp_counter);

		// Number of bytes written
		index_offset = index_offset + counter_read;
	}

	// Close file and filesystem
	f_close(&MyFile);

}
*/

/**
 * @brief  IAP write memory
 * @param  None
 * @retval None
 */
uint32_t COMMAND_DOWNLOAD(void) {
	FILINFO finfno = { 0 };
	FRESULT res = f_stat(UpdateFile, &finfno);
	if (res != FR_OK) {
		/* 'STM32.TXT' file Open for write Error */
		return DOWNLOAD_FILE_FAIL;
	}

	if (finfno.fsize <= (FLASH_SIZE - IAP_SIZE)) {
		/* Erase necessary page to download image */
		if (FLASH_If_Erase(APPLICATION_ADDRESS) != 0) {
			return DOWNLOAD_ERASE_FAIL;
		}

		/* Program flash memory */
		return COMMAND_ProgramFlashMemory();
	}
	return DOWNLOAD_OK;
}

/**
 * @brief  IAP jump to user program
 * @param  None
 * @retval None
 */
void COMMAND_JUMP(void) {
	/* Software reset */
	NVIC_SystemReset();
}

/**
 * @brief  COMMAND_ProgramFlashMemory
 * @param  None
 * @retval DOWNLOAD_OK: Download process completed
 * @retval DOWNLOAD_FILE_FAIL: not possible to read from opend file
 * @retval DOWNLOAD_WRITE_FAIL: not possible to write to FLASH
 */
uint32_t COMMAND_ProgramFlashMemory(void) {
	__IO uint32_t read_size = 0x00, tmp_read_size = 0x00;
	uint32_t read_flag = TRUE;

	/* Erase address init */
	LastPGAddress = APPLICATION_ADDRESS;

	/* While file still contain data */
	while (read_flag == TRUE) {

		memset(RAMBuf, 0xFF, BUFFERSIZE);

		/* Read maximum "BUFFERSIZE" Kbyte from the selected file  */
		if (f_read(&USERFile, RAMBuf, BUFFERSIZE, (UINT*) &read_size)
				!= FR_OK) {
			return DOWNLOAD_FILE_FAIL;
		}
		STATUS_LED_OFF;
		/* Temp variable */
		tmp_read_size = read_size;

		/* The read data < "BUFFERSIZE" Kbyte */
		if (tmp_read_size < BUFFERSIZE) {
			read_flag = FALSE;
		}

		/* Program flash memory */
		if (FLASH_If_Write(LastPGAddress, (uint32_t*) RAMBuf, read_size)
				!= FLASHIF_OK) {
			return DOWNLOAD_WRITE_FAIL;
		}
		STATUS_LED_ON;
		/* Update last programmed address value */
		LastPGAddress = LastPGAddress + tmp_read_size;
	}

	return DOWNLOAD_OK;
}

/**
 * @}
 */

/*******************(C)COPYRIGHT STMicroelectronics *****END OF FILE******/
