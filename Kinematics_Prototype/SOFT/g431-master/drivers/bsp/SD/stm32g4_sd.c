/**
 *******************************************************************************
 * @file	stm32g4_sd.c
 * @author	vchav
 * @date	Jul 12, 2024
 * @brief
 *******************************************************************************
 */

/**
 ******************************************************************************
 * @file    stm32_adafruit_sd.c
 * @author  MCD Application Team
 * @version V2.0.1
 * @date    04-November-2015
 * @brief   This file provides a set of functions needed to manage the SD card
 *          mounted on the Adafruit 1.8" TFT LCD shield (reference ID 802),
 *          that is used with the STM32 Nucleo board through SPI interface.
 *          It implements a high level communication layer for read and write
 *          from/to this memory. The needed STM32XXxx hardware resources (SPI and
 *          GPIO) are defined in stm32XXxx_nucleo.h file, and the initialization is
 *          performed in SD_IO_Init() function declared in stm32XXxx_nucleo.c
 *          file.
 *          You can easily tailor this driver to any other development board,
 *          by just adapting the defines for hardware resources and
 *          SD_IO_Init() function.
 *
 *          +-------------------------------------------------------+
 *          |                     Pin assignment                    |
 *          +-------------------------+---------------+-------------+
 *          |  STM32XXxx SPI Pins     |     SD        |    Pin      |
 *          +-------------------------+---------------+-------------+
 *          | SD_SPI_CS_PIN           |   ChipSelect  |    1        |
 *          | SD_SPI_MOSI_PIN / MOSI  |   DataIn      |    2        |
 *          |                         |   GND         |    3 (0 V)  |
 *          |                         |   VDD         |    4 (3.3 V)|
 *          | SD_SPI_SCK_PIN / SCLK   |   Clock       |    5        |
 *          |                         |   GND         |    6 (0 V)  |
 *          | SD_SPI_MISO_PIN / MISO  |   DataOut     |    7        |
 *          +-------------------------+---------------+-------------+
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

/* File Info : -----------------------------------------------------------------
								   User NOTES
1. How to use this driver:
--------------------------
   - This driver does not need a specific component driver for the micro SD device
	 to be included with.

2. Driver description:
---------------------
  + Initialization steps:
	 o Initialize the micro SD card using the BSP_SD_Init() function.
	 o Checking the SD card presence is not managed because SD detection pin is
	   not physically mapped on the Adafruit shield.
	 o The function BSP_SD_GetCardInfo() is used to get the micro SD card information
	   which is stored in the structure "SD_CardInfo".

  + Micro SD card operations
	 o The micro SD card can be accessed with read/write block(s) operations once
	   it is ready for access. The access can be performed in polling
	   mode by calling the functions BSP_SD_ReadBlocks()/BSP_SD_WriteBlocks()

	 o The SD erase block(s) is performed using the function BSP_SD_Erase() with
	   specifying the number of blocks to erase.
	 o The SD runtime status is returned when calling the function BSP_SD_GetStatus().

------------------------------------------------------------------------------*/

#include <config.h>
#if USE_SD_CARD
// #warning "Le module n'a pas �t� test� de mani�re concluente"
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stm32g4_gpio.h>
#include <stm32g431xx.h>
#include <stm32g4xx_hal_def.h>
#include <stm32g4xx_hal_gpio.h>
#include <stm32g4xx_hal_spi.h>
#include <sys/_stdint.h>
#include <SD/FatFs/src/ff.h>
#include <SD/FatFs/src/ff_gen_drv.h>
#include <SD/FatFs/src/integer.h>
#include "stm32g4_sd.h"
#include "stdlib.h"
#include "string.h"
#include "stm32g4_utils.h"
#include <stdio.h>
#include "FatFs/src/drivers/sd_diskio.h"
#include "stm32g4_spi.h"

char SD_path[4];

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
	uint8_t r1;
	uint8_t r2;
	uint8_t r3;
	uint8_t r4;
	uint8_t r5;
} SD_CmdAnswer_typedef;

/* Private define ------------------------------------------------------------*/
#define SD_DUMMY_BYTE 0xFF
#define SD_NO_RESPONSE_EXPECTED 0x80

#define SD_MAX_FRAME_LENGTH 17 /* Lenght = 16 + 1 */
#define SD_CMD_LENGTH 6

#define SD_MAX_TRY 100 /* Number of try */

#define SD_CSD_STRUCT_V1 0x2 /* CSD struct version V1 */
#define SD_CSD_STRUCT_V2 0x1 /* CSD struct version V2 */

/**
 * @brief  SD Control Interface pins
 */
#define SD_CS_PIN GPIO_PIN_0
#define SD_CS_GPIO_PORT GPIOB
#define SD_CS_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define SD_CS_GPIO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()
#define SD_SPI SPI1

/**
 * @brief  SD Control Lines management
 */
#define SD_CS_LOW() HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH() HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET)

/**
 * @brief  SD ansewer format
 */
typedef enum
{
	SD_ANSWER_R1_EXPECTED,
	SD_ANSWER_R1B_EXPECTED,
	SD_ANSWER_R2_EXPECTED,
	SD_ANSWER_R3_EXPECTED,
	SD_ANSWER_R4R5_EXPECTED,
	SD_ANSWER_R7_EXPECTED,
} SD_Answer_type;

/**
 * @brief  Start Data tokens:
 *         Tokens (necessary because at nop/idle (and CS active) only 0xff is
 *         on the data/command line)
 */
#define SD_TOKEN_START_DATA_SINGLE_BLOCK_READ 0xFE	  /* Data token start byte, Start Single Block Read */
#define SD_TOKEN_START_DATA_MULTIPLE_BLOCK_READ 0xFE  /* Data token start byte, Start Multiple Block Read */
#define SD_TOKEN_START_DATA_SINGLE_BLOCK_WRITE 0xFE	  /* Data token start byte, Start Single Block Write */
#define SD_TOKEN_START_DATA_MULTIPLE_BLOCK_WRITE 0xFD /* Data token start byte, Start Multiple Block Write */
#define SD_TOKEN_STOP_DATA_MULTIPLE_BLOCK_WRITE 0xFD  /* Data toke stop byte, Stop Multiple Block Write */

/**
 * @brief  Commands: CMDxx = CMD-number | 0x40
 */
#define SD_CMD_GO_IDLE_STATE 0		 /* CMD0 = 0x40  */
#define SD_CMD_SEND_OP_COND 1		 /* CMD1 = 0x41  */
#define SD_CMD_SEND_IF_COND 8		 /* CMD8 = 0x48  */
#define SD_CMD_SEND_CSD 9			 /* CMD9 = 0x49  */
#define SD_CMD_SEND_CID 10			 /* CMD10 = 0x4A */
#define SD_CMD_STOP_TRANSMISSION 12	 /* CMD12 = 0x4C */
#define SD_CMD_SEND_STATUS 13		 /* CMD13 = 0x4D */
#define SD_CMD_SET_BLOCKLEN 16		 /* CMD16 = 0x50 */
#define SD_CMD_READ_SINGLE_BLOCK 17	 /* CMD17 = 0x51 */
#define SD_CMD_READ_MULT_BLOCK 18	 /* CMD18 = 0x52 */
#define SD_CMD_SET_BLOCK_COUNT 23	 /* CMD23 = 0x57 */
#define SD_CMD_WRITE_SINGLE_BLOCK 24 /* CMD24 = 0x58 */
#define SD_CMD_WRITE_MULT_BLOCK 25	 /* CMD25 = 0x59 */
#define SD_CMD_PROG_CSD 27			 /* CMD27 = 0x5B */
#define SD_CMD_SET_WRITE_PROT 28	 /* CMD28 = 0x5C */
#define SD_CMD_CLR_WRITE_PROT 29	 /* CMD29 = 0x5D */
#define SD_CMD_SEND_WRITE_PROT 30	 /* CMD30 = 0x5E */
#define SD_CMD_SD_ERASE_GRP_START 32 /* CMD32 = 0x60 */
#define SD_CMD_SD_ERASE_GRP_END 33	 /* CMD33 = 0x61 */
#define SD_CMD_UNTAG_SECTOR 34		 /* CMD34 = 0x62 */
#define SD_CMD_ERASE_GRP_START 35	 /* CMD35 = 0x63 */
#define SD_CMD_ERASE_GRP_END 36		 /* CMD36 = 0x64 */
#define SD_CMD_UNTAG_ERASE_GROUP 37	 /* CMD37 = 0x65 */
#define SD_CMD_ERASE 38				 /* CMD38 = 0x66 */
#define SD_CMD_SD_APP_OP_COND 41	 /* CMD41 = 0x69 */
#define SD_CMD_APP_CMD 55			 /* CMD55 = 0x77 */
#define SD_CMD_READ_OCR 58			 /* CMD55 = 0x79 */

/**
 * @brief  SD reponses and error flags
 */
typedef enum
{
	/* R1 answer value */
	SD_R1_NO_ERROR = (0x00),
	SD_R1_IN_IDLE_STATE = (0x01),
	SD_R1_ERASE_RESET = (0x02),
	SD_R1_ILLEGAL_COMMAND = (0x04),
	SD_R1_COM_CRC_ERROR = (0x08),
	SD_R1_ERASE_SEQUENCE_ERROR = (0x10),
	SD_R1_ADDRESS_ERROR = (0x20),
	SD_R1_PARAMETER_ERROR = (0x40),

	/* R2 answer value */
	SD_R2_NO_ERROR = 0x00,
	SD_R2_CARD_LOCKED = 0x01,
	SD_R2_LOCKUNLOCK_ERROR = 0x02,
	SD_R2_ERROR = 0x04,
	SD_R2_CC_ERROR = 0x08,
	SD_R2_CARD_ECC_FAILED = 0x10,
	SD_R2_WP_VIOLATION = 0x20,
	SD_R2_ERASE_PARAM = 0x40,
	SD_R2_OUTOFRANGE = 0x80,

	/**
	 * @brief  Data response error
	 */
	SD_DATA_OK = (0x05),
	SD_DATA_CRC_ERROR = (0x0B),
	SD_DATA_WRITE_ERROR = (0x0D),
	SD_DATA_OTHER_ERROR = (0xFF)
} SD_Error;

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

__IO uint8_t SdStatus = SD_NOT_PRESENT;

/* flag_SDHC :
	  0 :  Standard capacity
	  1 : High capacity
*/
uint16_t flag_SDHC = 0;

/* Private function prototypes -----------------------------------------------*/
static void SD_IO_Init(void);
static HAL_StatusTypeDef SD_IO_WriteCmd(uint8_t Cmd, uint32_t Arg, uint8_t Crc, uint8_t Response);
static HAL_StatusTypeDef SD_IO_WaitResponse(uint8_t Response);
static void SD_IO_WriteDummy(void);
static uint8_t SD_IO_WriteByte(uint8_t Data);
static uint8_t SD_IO_ReadByte(void);
static uint8_t SD_IO_WriteReadByte(uint8_t Data);
static uint8_t SD_GetCIDRegister(SD_CID *Cid);
static uint8_t SD_GetCSDRegister(SD_CSD *Csd);
static uint8_t SD_GetDataResponse(void);
static uint8_t SD_GoIdleState(void);
static SD_CmdAnswer_typedef SD_SendCmd(uint8_t Cmd, uint32_t Arg, uint8_t Crc, uint8_t Answer);
static uint8_t SD_WaitData(uint8_t data);
static uint8_t SD_ReadData(void);

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Initializes the SD Card and put it into StandBy State (Ready for
 *         data transfer).
 * @retval None
 */
static void SD_IO_Init(void)
{
	GPIO_InitTypeDef gpioinitstruct = {0};
	uint8_t counter = 0;

	/* SD_CS_GPIO Periph clock enable */
	SD_CS_GPIO_CLK_ENABLE();

	/* Configure SD_CS_PIN pin: SD Card CS pin */
	gpioinitstruct.Pin = SD_CS_PIN;
	gpioinitstruct.Mode = GPIO_MODE_OUTPUT_PP;
	gpioinitstruct.Pull = GPIO_PULLUP;
	gpioinitstruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SD_CS_GPIO_PORT, &gpioinitstruct);

	/*------------Put SD in SPI mode--------------*/
	/* SD SPI Config */
	BSP_SPI_Init(SD_SPI, FULL_DUPLEX, MASTER, SPI_BAUDRATEPRESCALER_256); // Le choix � �t� fait de mettre aussi lent au moment du d�bogage

	/* SD chip select high */
	SD_CS_HIGH();

	/* Send dummy byte 0xFF, 10 times with CS high */
	/* Rise CS and MOSI for 80 clocks cycles */
	/*
	BSP_GPIO_pin_config(GPIOA, GPIO_PIN_5, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);
	BSP_GPIO_pin_config(GPIOA, GPIO_PIN_7, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1); //Les codes en commentaires ici permettent l'envoyer manuellement les dummy bytes pour ralentir un max
	*/
	for (counter = 0; counter <= 9 /* *8 */; counter++)
	{
		/* Send dummy byte 0xFF */
		SD_IO_WriteByte(SD_DUMMY_BYTE);
		/*
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
		Delay_us(10);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
		Delay_us(10);
		*/
	}
	/*
	BSP_GPIO_pin_config(GPIOA, GPIO_PIN_5, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF5_SPI1);
	BSP_GPIO_pin_config(GPIOA, GPIO_PIN_7, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF5_SPI1);
	*/
}

/**
 * @brief  Set the SD_CS pin.
 * @param  val: pin value.
 */
static void SD_IO_CSState(uint8_t val)
{
	if (val == 1)
	{
		SD_CS_HIGH();
	}
	else
	{
		SD_CS_LOW();
	}
}

/**
 * @brief  Write a byte on the SD.
 * @param  DataIn: byte to send.
 * @param  DataOut: byte to read
 * @param  DataLength: length of data
 */
static void SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
{
	/* Send the byte */
	uint16_t i;
	for (i = 0; i < DataLength; i++)
		DataOut[i] = (uint8_t)(BSP_SPI_WriteRead(SD_SPI, DataIn[i]));
}

/**
 * @brief  Writes a byte on the SD.
 * @param  Data: byte to send.
 * @retval None
 */
static uint8_t SD_IO_WriteByte(uint8_t Data)
{
	/* Send the byte */
	return BSP_SPI_WriteRead(SD_SPI, Data) & 0xFF;
}

static uint8_t SD_IO_WriteReadByte(uint8_t Data)
{
	/* Send the byte */
	uint8_t ret;
	SD_IO_WriteReadData(&Data, &ret, 1);
	return ret;
}

/**
 * @brief  Reads a byte from the SD.
 * @retval The received byte.
 */
static uint8_t SD_IO_ReadByte(void)
{
	uint8_t data = 0;

	/* Get the received data */
	data = BSP_SPI_WriteRead(SD_SPI, (uint8_t)0xFFFFFFFF);

	/* Return the shifted data */
	return data;
}

/**
 * @brief  Sends 5 bytes command to the SD card and get response
 * @param  Cmd: The user expected command to send to SD card.
 * @param  Arg: The command argument.
 * @param  Crc: The CRC.
 * @param  Response: Expected response from the SD card
 * @retval HAL_StatusTypeDef HAL Status
 */
HAL_StatusTypeDef SD_IO_WriteCmd(uint8_t Cmd, uint32_t Arg, uint8_t Crc, uint8_t response)
{
	uint32_t counter = 0x00;
	uint8_t frame[6] = {0};

	/* Prepare Frame to send */
	frame[0] = (Cmd | 0x40);		 /* Construct byte 1 */
	frame[1] = (uint8_t)(Arg >> 24); /* Construct byte 2 */
	frame[2] = (uint8_t)(Arg >> 16); /* Construct byte 3 */
	frame[3] = (uint8_t)(Arg >> 8);	 /* Construct byte 4 */
	frame[4] = (uint8_t)(Arg);		 /* Construct byte 5 */
	frame[5] = (Crc);				 /* Construct byte 6 */

	/* SD chip select low */
	SD_CS_LOW();

	/* Send Frame */
	for (counter = 0; counter < 6; counter++)
	{
		SD_IO_WriteByte(frame[counter]); /* Send the Cmd bytes */
	}

	if (response != SD_NO_RESPONSE_EXPECTED)
	{
		return SD_IO_WaitResponse(response);
	}

	return HAL_OK;
}

/**
 * @brief  Waits response from the SD card
 * @param  Response: Expected response from the SD card
 * @retval HAL_StatusTypeDef HAL Status
 */
HAL_StatusTypeDef SD_IO_WaitResponse(uint8_t Response)
{
	uint32_t timeout = 0xFFFF;

	/* Check if response is got or a timeout is happen */
	while ((SD_IO_ReadByte() != Response) && timeout)
	{
		timeout--;
	}

	if (timeout == 0)
	{
		/* After time out */
		return HAL_TIMEOUT;
	}
	else
	{
		/* Right response got */
		return HAL_OK;
	}
}

/**
 * @brief  Sends dummy byte with CS High
 * @retval None
 */
void SD_IO_WriteDummy(void)
{
	/* SD chip select high */
	SD_CS_HIGH();

	/* Send Dummy byte 0xFF */
	SD_IO_WriteByte(SD_DUMMY_BYTE);
}

/**
 * @brief  Initializes the SD/SD communication.
 * @param  None
 * @retval The SD Response:
 *         - MSD_ERROR: Sequence failed
 *         - MSD_OK: Sequence succeed
 */
uint8_t BSP_SD_Init(void)
{
	/* Configure IO functionalities for SD pin */
	SD_IO_Init();

	/* SD detection pin is not physically mapped on the Adafruit shield */
	SdStatus = SD_PRESENT;

	/* SD initialized and set to SPI mode properly */
	return SD_GoIdleState();
}

/**
 * @brief  Returns information about specific card.
 * @param  pCardInfo: Pointer to a SD_CardInfo structure that contains all SD
 *         card information.
 * @retval The SD Response:
 *         - MSD_ERROR: Sequence failed
 *         - MSD_OK: Sequence succeed
 */
uint8_t BSP_SD_GetCardInfo(SD_CardInfo *pCardInfo)
{
	uint8_t status;

	status = SD_GetCSDRegister(&(pCardInfo->Csd));
	status |= SD_GetCIDRegister(&(pCardInfo->Cid));
	if (flag_SDHC == 1)
	{
		pCardInfo->CardBlockSize = 512;
		pCardInfo->CardCapacity = (uint32_t)((pCardInfo->Csd.version.v2.DeviceSize + 1) * pCardInfo->CardBlockSize);
	}
	else
	{
		pCardInfo->CardCapacity = (uint32_t)((pCardInfo->Csd.version.v1.DeviceSize + 1));
		pCardInfo->CardCapacity *= (uint32_t)((1 << (pCardInfo->Csd.version.v1.DeviceSizeMul + 2)));
		pCardInfo->CardBlockSize = (uint32_t)(1 << (pCardInfo->Csd.RdBlockLen));
		pCardInfo->CardCapacity *= pCardInfo->CardBlockSize;
	}

	return status;
}

/**
 * @brief  Reads block(s) from a specified address in the SD card, in polling mode.
 * @param  pData: Pointer to the buffer that will contain the data to transmit
 * @param  ReadAddr: Address from where data is to be read
 * @param  BlockSize: SD card data block size, that should be 512
 * @param  NumOfBlocks: Number of SD blocks to read
 * @retval SD status
 */
uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
	uint32_t offset = 0;
	uint8_t retr = BSP_SD_ERROR;
	uint8_t *ptr = NULL;
	SD_CmdAnswer_typedef response;

	/* Send CMD16 (SD_CMD_SET_BLOCKLEN) to set the size of the block and
	 Check if the SD acknowledged the set block length command: R1 response (0x00: no errors) */
	response = SD_SendCmd(SD_CMD_SET_BLOCKLEN, BlockSize, 0xFF, SD_ANSWER_R1_EXPECTED);
	SD_IO_CSState(1);
	SD_IO_WriteByte(SD_DUMMY_BYTE);
	if (response.r1 != SD_R1_NO_ERROR)
	{
		// goto error;
		return ret_error(retr, ptr);
	}

	ptr = malloc(sizeof(uint8_t) * BlockSize);
	if (ptr == NULL)
	{
		// goto error;
		return ret_error(retr, ptr);
	}
	memset(ptr, SD_DUMMY_BYTE, sizeof(uint8_t) * BlockSize);

	/* Data transfer */
	while (NumberOfBlocks--)
	{
		/* Send CMD17 (SD_CMD_READ_SINGLE_BLOCK) to read one block */
		/* Check if the SD acknowledged the read block command: R1 response (0x00: no errors) */
		response = SD_SendCmd(SD_CMD_READ_SINGLE_BLOCK, (ReadAddr + offset) / (flag_SDHC == 1 ? BlockSize : 1), 0xFF, SD_ANSWER_R1_EXPECTED);
		if (response.r1 != SD_R1_NO_ERROR)
		{
			// goto error;
			return ret_error(retr, ptr);
		}

		/* Now look for the data token to signify the start of the data */
		if (SD_WaitData(SD_TOKEN_START_DATA_SINGLE_BLOCK_READ) == BSP_SD_OK)
		{
			/* Read the SD block data : read NumByteToRead data */
			SD_IO_WriteReadData(ptr, (uint8_t *)pData + offset, BlockSize);

			/* Set next read address*/
			offset += BlockSize;
			/* get CRC bytes (not really needed by us, but required by SD) */
			SD_IO_WriteByte(SD_DUMMY_BYTE);
			SD_IO_WriteByte(SD_DUMMY_BYTE);
		}
		else
		{
			// goto error;
			return ret_error(retr, ptr);
		}

		/* End the command data read cycle */
		SD_IO_CSState(1);
		SD_IO_WriteByte(SD_DUMMY_BYTE);
	}

	retr = BSP_SD_OK;

	SD_IO_CSState(1);
	SD_IO_WriteByte(SD_DUMMY_BYTE);
	if (ptr != NULL)
		free(ptr);
	return retr;
	/*
  error :
	// Send dummy byte: 8 Clock pulses of delay
	SD_IO_CSState(1);
	SD_IO_WriteByte(SD_DUMMY_BYTE);
	if(ptr != NULL) free(ptr);

	// Return the reponse
	return retr;
	*/
}

uint8_t ret_error(uint8_t retr, uint8_t *ptr)
{
	/* Send dummy byte: 8 Clock pulses of delay */
	SD_IO_CSState(1);
	SD_IO_WriteByte(SD_DUMMY_BYTE);
	if (ptr != NULL)
		free(ptr);

	/* Return the reponse */
	return retr;
}

/**
 * @brief  Writes block(s) to a specified address in the SD card, in polling mode.
 * @param  pData: Pointer to the buffer that will contain the data to transmit
 * @param  WriteAddr: Address from where data is to be written
 * @param  BlockSize: SD card data block size, that should be 512
 * @param  NumOfBlocks: Number of SD blocks to write
 * @retval SD status
 */
uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
	uint32_t offset = 0;
	uint8_t retr = BSP_SD_ERROR;
	uint8_t *ptr = NULL;
	SD_CmdAnswer_typedef response;

	/* Send CMD16 (SD_CMD_SET_BLOCKLEN) to set the size of the block and
	Check if the SD acknowledged the set block length command: R1 response (0x00: no errors) */
	response = SD_SendCmd(SD_CMD_SET_BLOCKLEN, BlockSize, 0xFF, SD_ANSWER_R1_EXPECTED);
	SD_IO_CSState(1);
	SD_IO_WriteByte(SD_DUMMY_BYTE);
	if (response.r1 != SD_R1_NO_ERROR)
	{
		goto error;
	}

	ptr = malloc(sizeof(uint8_t) * BlockSize);
	if (ptr == NULL)
	{
		goto error;
	}

	/* Data transfer */
	while (NumberOfBlocks--)
	{
		/* Send CMD24 (SD_CMD_WRITE_SINGLE_BLOCK) to write blocks  and
		Check if the SD acknowledged the write block command: R1 response (0x00: no errors) */
		response = SD_SendCmd(SD_CMD_WRITE_SINGLE_BLOCK, (WriteAddr + offset) / (flag_SDHC == 1 ? BlockSize : 1), 0xFF, SD_ANSWER_R1_EXPECTED);
		if (response.r1 != SD_R1_NO_ERROR)
		{
			goto error;
		}

		/* Send dummy byte for NWR timing : one byte between CMDWRITE and TOKEN */
		SD_IO_WriteByte(SD_DUMMY_BYTE);
		SD_IO_WriteByte(SD_DUMMY_BYTE);

		/* Send the data token to signify the start of the data */
		SD_IO_WriteByte(SD_TOKEN_START_DATA_SINGLE_BLOCK_WRITE);

		/* Write the block data to SD */
		SD_IO_WriteReadData((uint8_t *)pData + offset, ptr, BlockSize);

		/* Set next write address */
		offset += BlockSize;

		/* Put CRC bytes (not really needed by us, but required by SD) */
		SD_IO_WriteByte(SD_DUMMY_BYTE);
		SD_IO_WriteByte(SD_DUMMY_BYTE);

		/* Read data response */
		if (SD_GetDataResponse() != SD_DATA_OK)
		{
			/* Set response value to failure */
			goto error;
		}

		SD_IO_CSState(1);
		SD_IO_WriteByte(SD_DUMMY_BYTE);
	}
	retr = BSP_SD_OK;

error:
	if (ptr != NULL)
		free(ptr);
	/* Send dummy byte: 8 Clock pulses of delay */
	SD_IO_CSState(1);
	SD_IO_WriteByte(SD_DUMMY_BYTE);

	/* Return the reponse */
	return retr;
}

/**
 * @brief  Erases the specified memory area of the given SD card.
 * @param  StartAddr: Start byte address
 * @param  EndAddr: End byte address
 * @retval SD status
 */
uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr)
{
	uint8_t retr = BSP_SD_ERROR;
	SD_CmdAnswer_typedef response;

	/* Send CMD32 (Erase group start) and check if the SD acknowledged the erase command: R1 response (0x00: no errors) */
	response = SD_SendCmd(SD_CMD_SD_ERASE_GRP_START, StartAddr, 0xFF, SD_ANSWER_R1_EXPECTED);
	SD_IO_CSState(1);
	SD_IO_WriteByte(SD_DUMMY_BYTE);
	if (response.r1 == SD_R1_NO_ERROR)
	{
		/* Send CMD33 (Erase group end) and Check if the SD acknowledged the erase command: R1 response (0x00: no errors) */
		response = SD_SendCmd(SD_CMD_SD_ERASE_GRP_END, EndAddr, 0xFF, SD_ANSWER_R1_EXPECTED);
		SD_IO_CSState(1);
		SD_IO_WriteByte(SD_DUMMY_BYTE);
		if (response.r1 == SD_R1_NO_ERROR)
		{
			/* Send CMD38 (Erase) and Check if the SD acknowledged the erase command: R1 response (0x00: no errors) */
			response = SD_SendCmd(SD_CMD_ERASE, 0, 0xFF, SD_ANSWER_R1B_EXPECTED);
			if (response.r1 == SD_R1_NO_ERROR)
			{
				retr = BSP_SD_OK;
			}
			SD_IO_CSState(1);
			SD_IO_WriteByte(SD_DUMMY_BYTE);
		}
	}

	/* Return the reponse */
	return retr;
}

/**
 * @brief  Returns the SD status.
 * @param  None
 * @retval The SD status.
 */
uint8_t BSP_SD_GetStatus(void)
{
	SD_CmdAnswer_typedef retr;

	/* Send CMD13 (SD_SEND_STATUS) to get SD status */
	retr = SD_SendCmd(SD_CMD_SEND_STATUS, 0, 0xFF, SD_ANSWER_R2_EXPECTED);
	SD_IO_CSState(1);
	SD_IO_WriteByte(SD_DUMMY_BYTE);

	/* Find SD status according to card state */
	if ((retr.r1 == SD_R1_NO_ERROR) && (retr.r2 == SD_R2_NO_ERROR))
	{
		return BSP_SD_OK;
	}

	return BSP_SD_ERROR;
}

/**
 * @brief  Reads the SD card SCD register.
 *         Reading the contents of the CSD register in SPI mode is a simple
 *         read-block transaction.
 * @param  Csd: pointer on an SCD register structure
 * @retval SD status
 */
uint8_t SD_GetCSDRegister(SD_CSD *Csd)
{
	uint16_t counter = 0;
	uint8_t CSD_Tab[16];
	uint8_t retr = BSP_SD_ERROR;
	SD_CmdAnswer_typedef response;

	/* Send CMD9 (CSD register) or CMD10(CSD register) and Wait for response in the R1 format (0x00 is no errors) */
	response = SD_SendCmd(SD_CMD_SEND_CSD, 0, 0xFF, SD_ANSWER_R1_EXPECTED);
	if (response.r1 == SD_R1_NO_ERROR)
	{
		if (SD_WaitData(SD_TOKEN_START_DATA_SINGLE_BLOCK_READ) == BSP_SD_OK)
		{
			for (counter = 0; counter < 16; counter++)
			{
				/* Store CSD register value on CSD_Tab */
				CSD_Tab[counter] = SD_IO_WriteByte(SD_DUMMY_BYTE);
			}

			/* Get CRC bytes (not really needed by us, but required by SD) */
			SD_IO_WriteByte(SD_DUMMY_BYTE);
			SD_IO_WriteByte(SD_DUMMY_BYTE);

			/*************************************************************************
			CSD header decoding
			*************************************************************************/

			/* Byte 0 */
			Csd->CSDStruct = (CSD_Tab[0] & 0xC0) >> 6;
			Csd->Reserved1 = CSD_Tab[0] & 0x3F;

			/* Byte 1 */
			Csd->TAAC = CSD_Tab[1];

			/* Byte 2 */
			Csd->NSAC = CSD_Tab[2];

			/* Byte 3 */
			Csd->MaxBusClkFrec = CSD_Tab[3];

			/* Byte 4/5 */
			Csd->CardComdClasses = (uint16_t)((CSD_Tab[4] << 4) | ((CSD_Tab[5] & 0xF0) >> 4));
			Csd->RdBlockLen = (uint8_t)(CSD_Tab[5] & 0x0F);

			/* Byte 6 */
			Csd->PartBlockRead = (uint8_t)((CSD_Tab[6] & 0x80) >> 7);
			Csd->WrBlockMisalign = (uint8_t)((CSD_Tab[6] & 0x40) >> 6);
			Csd->RdBlockMisalign = (uint8_t)((CSD_Tab[6] & 0x20) >> 5);
			Csd->DSRImpl = (uint8_t)((CSD_Tab[6] & 0x10) >> 4);

			/*************************************************************************
			CSD v1/v2 decoding
			*************************************************************************/

			if (flag_SDHC == 0)
			{
				Csd->version.v1.Reserved1 = ((CSD_Tab[6] & 0x0C) >> 2);

				Csd->version.v1.DeviceSize = ((CSD_Tab[6] & 0x03) << 10) | (CSD_Tab[7] << 2) | ((CSD_Tab[8] & 0xC0) >> 6);
				Csd->version.v1.MaxRdCurrentVDDMin = (CSD_Tab[8] & 0x38) >> 3;
				Csd->version.v1.MaxRdCurrentVDDMax = (CSD_Tab[8] & 0x07);
				Csd->version.v1.MaxWrCurrentVDDMin = (CSD_Tab[9] & 0xE0) >> 5;
				Csd->version.v1.MaxWrCurrentVDDMax = (CSD_Tab[9] & 0x1C) >> 2;
				Csd->version.v1.DeviceSizeMul = ((CSD_Tab[9] & 0x03) << 1) | ((CSD_Tab[10] & 0x80) >> 7);
			}
			else
			{
				Csd->version.v2.Reserved1 = ((CSD_Tab[6] & 0x0F) << 2) | ((CSD_Tab[7] & 0xC0) >> 6);
				Csd->version.v2.DeviceSize = ((CSD_Tab[7] & 0x3F) << 16) | (CSD_Tab[8] << 8) | CSD_Tab[9];
				Csd->version.v2.Reserved2 = ((CSD_Tab[10] & 0x80) >> 8);
			}

			Csd->EraseSingleBlockEnable = (CSD_Tab[10] & 0x40) >> 6;
			Csd->EraseSectorSize = ((CSD_Tab[10] & 0x3F) << 1) | ((CSD_Tab[11] & 0x80) >> 7);
			Csd->WrProtectGrSize = (CSD_Tab[11] & 0x7F);
			Csd->WrProtectGrEnable = (CSD_Tab[12] & 0x80) >> 7;
			Csd->Reserved2 = (CSD_Tab[12] & 0x60) >> 5;
			Csd->WrSpeedFact = (CSD_Tab[12] & 0x1C) >> 2;
			Csd->MaxWrBlockLen = ((CSD_Tab[12] & 0x03) << 2) | ((CSD_Tab[13] & 0xC0) >> 6);
			Csd->WriteBlockPartial = (CSD_Tab[13] & 0x20) >> 5;
			Csd->Reserved3 = (CSD_Tab[13] & 0x1F);
			Csd->FileFormatGrouop = (CSD_Tab[14] & 0x80) >> 7;
			Csd->CopyFlag = (CSD_Tab[14] & 0x40) >> 6;
			Csd->PermWrProtect = (CSD_Tab[14] & 0x20) >> 5;
			Csd->TempWrProtect = (CSD_Tab[14] & 0x10) >> 4;
			Csd->FileFormat = (CSD_Tab[14] & 0x0C) >> 2;
			Csd->Reserved4 = (CSD_Tab[14] & 0x03);
			Csd->crc = (CSD_Tab[15] & 0xFE) >> 1;
			Csd->Reserved5 = (CSD_Tab[15] & 0x01);

			retr = BSP_SD_OK;
		}
	}

	/* Send dummy byte: 8 Clock pulses of delay */
	SD_IO_CSState(1);
	SD_IO_WriteByte(SD_DUMMY_BYTE);

	/* Return the reponse */
	return retr;
}

/**
 * @brief  Reads the SD card CID register.
 *         Reading the contents of the CID register in SPI mode is a simple
 *         read-block transaction.
 * @param  Cid: pointer on an CID register structure
 * @retval SD status
 */
uint8_t SD_GetCIDRegister(SD_CID *Cid)
{
	uint32_t counter = 0;
	uint8_t retr = BSP_SD_ERROR;
	uint8_t CID_Tab[16];
	SD_CmdAnswer_typedef response;

	/* Send CMD10 (CID register) and Wait for response in the R1 format (0x00 is no errors) */
	response = SD_SendCmd(SD_CMD_SEND_CID, 0, 0xFF, SD_ANSWER_R1_EXPECTED);
	if (response.r1 == SD_R1_NO_ERROR)
	{
		if (SD_WaitData(SD_TOKEN_START_DATA_SINGLE_BLOCK_READ) == BSP_SD_OK)
		{
			/* Store CID register value on CID_Tab */
			for (counter = 0; counter < 16; counter++)
			{
				CID_Tab[counter] = SD_IO_WriteReadByte(SD_DUMMY_BYTE);
			}

			/* Get CRC bytes (not really needed by us, but required by SD) */
			SD_IO_WriteByte(SD_DUMMY_BYTE);
			SD_IO_WriteByte(SD_DUMMY_BYTE);

			/* Byte 0 */
			Cid->ManufacturerID = CID_Tab[0];

			/* Byte 1 */
			Cid->OEM_AppliID = (uint16_t)(CID_Tab[1] << 8);

			/* Byte 2 */
			Cid->OEM_AppliID |= (uint16_t)(CID_Tab[2]);

			/* Byte 3 */
			Cid->ProdName1 = (uint32_t)(CID_Tab[3] << 24);

			/* Byte 4 */
			Cid->ProdName1 |= (uint32_t)(CID_Tab[4] << 16);

			/* Byte 5 */
			Cid->ProdName1 |= (uint32_t)(CID_Tab[5] << 8);

			/* Byte 6 */
			Cid->ProdName1 |= (uint32_t)(CID_Tab[6]);

			/* Byte 7 */
			Cid->ProdName2 = CID_Tab[7];

			/* Byte 8 */
			Cid->ProdRev = CID_Tab[8];

			/* Byte 9 */
			Cid->ProdSN = (uint32_t)(CID_Tab[9] << 24);

			/* Byte 10 */
			Cid->ProdSN |= (uint32_t)(CID_Tab[10] << 16);

			/* Byte 11 */
			Cid->ProdSN |= (uint32_t)(CID_Tab[11] << 8);

			/* Byte 12 */
			Cid->ProdSN |= (uint32_t)(CID_Tab[12]);

			/* Byte 13 */
			Cid->Reserved1 |= (CID_Tab[13] & 0xF0) >> 4;
			Cid->ManufactDate = (CID_Tab[13] & 0x0F) << 8;

			/* Byte 14 */
			Cid->ManufactDate |= (uint16_t)(CID_Tab[14]);

			/* Byte 15 */
			Cid->CID_CRC = (CID_Tab[15] & 0xFE) >> 1;
			Cid->Reserved2 = 1;

			retr = BSP_SD_OK;
		}
	}

	/* Send dummy byte: 8 Clock pulses of delay */
	SD_IO_CSState(1);
	SD_IO_WriteByte(SD_DUMMY_BYTE);

	/* Return the reponse */
	return retr;
}

/**
 * @brief  Sends 5 bytes command to the SD card and get response
 * @param  Cmd: The user expected command to send to SD card.
 * @param  Arg: The command argument.
 * @param  Crc: The CRC.
 * @param  Answer: SD_ANSWER_NOT_EXPECTED or SD_ANSWER_EXPECTED
 * @retval SD status
 */
SD_CmdAnswer_typedef SD_SendCmd(uint8_t Cmd, uint32_t Arg, uint8_t Crc, uint8_t Answer)
{
	uint8_t frame[SD_CMD_LENGTH], frameout[SD_CMD_LENGTH];
	SD_CmdAnswer_typedef retr = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	/* R1 Lenght = NCS(0)+ 6 Bytes command + NCR(min1 max8) + 1 Bytes answer + NEC(0) = 15bytes */
	/* R1b identical to R1 + Busy information                                                   */
	/* R2 Lenght = NCS(0)+ 6 Bytes command + NCR(min1 max8) + 2 Bytes answer + NEC(0) = 16bytes */

	/* Prepare Frame to send */
	frame[0] = (Cmd | 0x40);		 /* Construct byte 1 */
	frame[1] = (uint8_t)(Arg >> 24); /* Construct byte 2 */
	frame[2] = (uint8_t)(Arg >> 16); /* Construct byte 3 */
	frame[3] = (uint8_t)(Arg >> 8);	 /* Construct byte 4 */
	frame[4] = (uint8_t)(Arg);		 /* Construct byte 5 */
	frame[5] = (Crc | 0x01);		 /* Construct byte 6 */
	/* Send the command */
	SD_IO_CSState(0);
	SD_IO_WriteReadData(frame, frameout, SD_CMD_LENGTH); /* Send the Cmd bytes */
	switch (Answer)
	{
	case SD_ANSWER_R1_EXPECTED:
		retr.r1 = SD_ReadData();
		break;
	case SD_ANSWER_R1B_EXPECTED:
		retr.r1 = SD_ReadData();
		retr.r2 = SD_IO_WriteReadByte(SD_DUMMY_BYTE);
		/* Set CS High */
		SD_IO_CSState(1);
		HAL_Delay(1);
		/* Set CS Low */
		SD_IO_CSState(0);

		/* Wait IO line return 0xFF */
		while (SD_IO_WriteReadByte(SD_DUMMY_BYTE) != 0xFF)
			;
		break;
	case SD_ANSWER_R2_EXPECTED:
		retr.r1 = SD_ReadData();
		retr.r2 = SD_IO_WriteReadByte(SD_DUMMY_BYTE);
		break;
	case SD_ANSWER_R3_EXPECTED:
	case SD_ANSWER_R7_EXPECTED:
		retr.r1 = SD_ReadData();
		retr.r2 = SD_IO_WriteReadByte(SD_DUMMY_BYTE);
		retr.r3 = SD_IO_WriteReadByte(SD_DUMMY_BYTE);
		retr.r4 = SD_IO_WriteReadByte(SD_DUMMY_BYTE);
		retr.r5 = SD_IO_WriteReadByte(SD_DUMMY_BYTE);
		break;
	default:
		break;
	}
	return retr;
}

/**
 * @brief  Gets the SD card data response and check the busy flag.
 * @param  None
 * @retval The SD status: Read data response xxx0<status>1
 *         - status 010: Data accecpted
 *         - status 101: Data rejected due to a crc error
 *         - status 110: Data rejected due to a Write error.
 *         - status 111: Data rejected due to other error.
 */
uint8_t SD_GetDataResponse(void)
{
	uint8_t dataresponse;
	uint8_t rvalue = SD_DATA_OTHER_ERROR;

	dataresponse = SD_IO_WriteReadByte(SD_DUMMY_BYTE);
	SD_IO_WriteByte(SD_DUMMY_BYTE); /* read the busy response byte*/

	/* Mask unused bits */
	switch (dataresponse & 0x1F)
	{
	case SD_DATA_OK:
		rvalue = SD_DATA_OK;

		/* Set CS High */
		SD_IO_CSState(1);
		/* Set CS Low */
		SD_IO_CSState(0);

		/* Wait IO line return 0xFF */
		while (SD_IO_WriteByte(SD_DUMMY_BYTE) != 0xFF)
			;
		break;
	case SD_DATA_CRC_ERROR:
		rvalue = SD_DATA_CRC_ERROR;
		break;
	case SD_DATA_WRITE_ERROR:
		rvalue = SD_DATA_WRITE_ERROR;
		break;
	default:
		break;
	}

	/* Return response */
	return rvalue;
}

/**
 * @brief  Put the SD in Idle state.
 * @param  None
 * @retval SD status
 */
uint8_t SD_GoIdleState(void)
{
	SD_CmdAnswer_typedef response;
	__IO uint8_t counter = 0;
	/* Send CMD0 (SD_CMD_GO_IDLE_STATE) to put SD in SPI mode and
	wait for In Idle State Response (R1 Format) equal to 0x01 */
	do
	{
		counter++;
		response = SD_SendCmd(SD_CMD_GO_IDLE_STATE, 0, 0x95, SD_ANSWER_R1_EXPECTED);
		SD_IO_CSState(1);
		SD_IO_WriteByte(SD_DUMMY_BYTE);

		if (counter >= SD_MAX_TRY)
		{
			return BSP_SD_ERROR;
		}
	} while (response.r1 != SD_R1_IN_IDLE_STATE);

	/* Send CMD8 (SD_CMD_SEND_IF_COND) to check the power supply status
	and wait until response (R7 Format) equal to 0xAA and */
	response = SD_SendCmd(SD_CMD_SEND_IF_COND, 0x1AA, 0x87, SD_ANSWER_R7_EXPECTED);
	SD_IO_CSState(1);
	SD_IO_WriteByte(SD_DUMMY_BYTE);
	if ((response.r1 & SD_R1_ILLEGAL_COMMAND) == SD_R1_ILLEGAL_COMMAND)
	{
		/* initialise card V1 */
		do
		{
			/* initialise card V1 */
			/* Send CMD55 (SD_CMD_APP_CMD) before any ACMD command: R1 response (0x00: no errors) */
			response = SD_SendCmd(SD_CMD_APP_CMD, 0x00000000, 0xFF, SD_ANSWER_R1_EXPECTED);
			SD_IO_CSState(1);
			SD_IO_WriteByte(SD_DUMMY_BYTE);

			/* Send ACMD41 (SD_CMD_SD_APP_OP_COND) to initialize SDHC or SDXC cards: R1 response (0x00: no errors) */
			response = SD_SendCmd(SD_CMD_SD_APP_OP_COND, 0x00000000, 0xFF, SD_ANSWER_R1_EXPECTED);
			SD_IO_CSState(1);
			SD_IO_WriteByte(SD_DUMMY_BYTE);
		} while (response.r1 == SD_R1_IN_IDLE_STATE);
		flag_SDHC = 0;
	}
	else if (response.r1 == SD_R1_IN_IDLE_STATE)
	{
		/* initialise card V2 */
		do
		{
			/* Send CMD55 (SD_CMD_APP_CMD) before any ACMD command: R1 response (0x00: no errors) */
			response = SD_SendCmd(SD_CMD_APP_CMD, 0, 0xFF, SD_ANSWER_R1_EXPECTED);
			SD_IO_CSState(1);
			SD_IO_WriteByte(SD_DUMMY_BYTE);

			/* Send ACMD41 (SD_CMD_SD_APP_OP_COND) to initialize SDHC or SDXC cards: R1 response (0x00: no errors) */
			response = SD_SendCmd(SD_CMD_SD_APP_OP_COND, 0x40000000, 0xFF, SD_ANSWER_R1_EXPECTED);
			SD_IO_CSState(1);
			SD_IO_WriteByte(SD_DUMMY_BYTE);
		} while (response.r1 == SD_R1_IN_IDLE_STATE);

		if ((response.r1 & SD_R1_ILLEGAL_COMMAND) == SD_R1_ILLEGAL_COMMAND)
		{
			do
			{
				/* Send CMD55 (SD_CMD_APP_CMD) before any ACMD command: R1 response (0x00: no errors) */
				response = SD_SendCmd(SD_CMD_APP_CMD, 0, 0xFF, SD_ANSWER_R1_EXPECTED);
				SD_IO_CSState(1);
				SD_IO_WriteByte(SD_DUMMY_BYTE);
				if (response.r1 != SD_R1_IN_IDLE_STATE)
				{
					return BSP_SD_ERROR;
				}
				/* Send ACMD41 (SD_CMD_SD_APP_OP_COND) to initialize SDHC or SDXC cards: R1 response (0x00: no errors) */
				response = SD_SendCmd(SD_CMD_SD_APP_OP_COND, 0x00000000, 0xFF, SD_ANSWER_R1_EXPECTED);
				SD_IO_CSState(1);
				SD_IO_WriteByte(SD_DUMMY_BYTE);
			} while (response.r1 == SD_R1_IN_IDLE_STATE);
		}

		/* Send CMD58 (SD_CMD_READ_OCR) to initialize SDHC or SDXC cards: R3 response (0x00: no errors) */
		response = SD_SendCmd(SD_CMD_READ_OCR, 0x00000000, 0xFF, SD_ANSWER_R3_EXPECTED);
		SD_IO_CSState(1);
		SD_IO_WriteByte(SD_DUMMY_BYTE);
		if (response.r1 != SD_R1_NO_ERROR)
		{
			return BSP_SD_ERROR;
		}
		flag_SDHC = (response.r2 & 0x40) >> 6;
	}
	else
	{
		return BSP_SD_ERROR;
	}

	return BSP_SD_OK;
}

/**
 * @brief  Waits a data until a value different from SD_DUMMY_BITE
 * @param  None
 * @retval the value read
 */
uint8_t SD_ReadData(void)
{
	uint8_t timeout = 0x08;
	uint8_t readvalue;

	/* Check if response is got or a timeout is happen */
	do
	{
		readvalue = SD_IO_WriteByte(SD_DUMMY_BYTE);
		timeout--;
	} while ((readvalue == SD_DUMMY_BYTE) && timeout);

	/* Right response got */
	return readvalue;
}

/**
 * @brief  Waits a data from the SD card
 * @param  data : Expected data from the SD card
 * @retval BSP_SD_OK or BSP_SD_TIMEOUT
 */
uint8_t SD_WaitData(uint8_t data)
{
	uint16_t timeout = 0xFFFF;
	uint8_t readvalue;

	/* Check if response is got or a timeout is happen */

	do
	{
		readvalue = SD_IO_WriteByte(SD_DUMMY_BYTE);
		timeout--;
	} while ((readvalue != data) && timeout);

	if (timeout == 0)
	{
		/* After time out */
		return BSP_SD_TIMEOUT;
	}

	/* Right response got */
	return BSP_SD_OK;
}

#define display printf
const char *datas_to_write = "test_datas\n";
const char *path = "0:test.txt";
FATFS SDFatFs;

void verbose_fresult(FRESULT rc)
{
	const char *p;
	static const char str[] =
		"OK\0"
		"NOT_READY\0"
		"NO_FILE\0"
		"FR_NO_PATH\0"
		"INVALID_NAME\0"
		"INVALID_DRIVE\0"
		"DENIED\0"
		"EXIST\0"
		"RW_ERROR\0"
		"WRITE_PROTECTED\0"
		"NOT_ENABLED\0"
		"NO_FILESYSTEM\0"
		"INVALID_OBJECT\0"
		"MKFS_ABORTED\0";
	FRESULT i;

	for (p = str, i = 0; i != rc && *p; i++)
	{
		while (*p++)
			;
	}
	debug_printf("rc=%u FR_%s\n", (UINT)rc, p);
}

// Fonction pour tester la librairie d'acces a la carte SD ou a une cle USB...
running_t BSP_SD_demo_state_machine(bool ask_for_finish)
{
	typedef enum
	{
		INIT = 0,
		MOUNT,
		WRITE_TEST_FILE,
		READ_TEST_FILE,
		DELETE_TEST_FILE,
		END_WITH_ERROR,
		IDLE
	} state_e;

	static state_e state = INIT;
	FRESULT res = FR_OK;
	static FIL file;
	running_t ret;
	ret = IN_PROGRESS;

	switch (state)
	{
	case INIT:

		if (BSP_SD_Init() == MSD_OK) ///*disk_initialize(PD_SD) == 0*/)
		{
			debug_printf("SD Card : module initialized\n");
			state = MOUNT;
		}
		else
		{
			debug_printf("Fail to initialize disk\n"); // le dernier test tombait tout le temps ici
			state = END_WITH_ERROR;
		}
		break;
	case MOUNT:
	{

		FATFS *fs;
		uint32_t p2;
		state = END_WITH_ERROR; // On fait la supposition que ça ne va pas marcher.... si tout se passe bien, on ira faire la suite.
		if (FATFS_LinkDriver(&SD_Driver, SD_path) != 0)
		{
			display("SD Card : NOT mounted\n");
		}
		else
		{
			res = f_mount(&SDFatFs, (TCHAR const *)SD_path, 0);
			if (res == FR_OK)
			{
				res = f_getfree("", (DWORD *)&p2, &fs);
				// res = f_mkfs((TCHAR const*)SDPath, 0, 0);

				if (res == FR_OK)
				{
					debug_printf("FAT type = %u (%s)\nBytes/Cluster = %lu\nNumber of FATs = %u\n"
								 "Root DIR entries = %u\nSectors/FAT = %lu\nNumber of clusters = %lu\n"
								 "FAT start (lba) = %lu\nDIR start (lba,clustor) = %lu\nData start (lba) = %lu\n\n",
								 (WORD)fs->fs_type,
								 (fs->fs_type == FS_FAT12) ? "FAT12" : (fs->fs_type == FS_FAT16) ? "FAT16"
																								 : "FAT32",
								 (DWORD)fs->csize * 512, (WORD)fs->n_fats,
								 fs->n_rootdir, fs->fsize, (DWORD)fs->n_fatent - 2,
								 fs->fatbase, fs->dirbase, fs->database);
					display("SD Card : mounted\n");
					state = WRITE_TEST_FILE;
				}
			}
			if (res != FR_OK)
				verbose_fresult(res);
		}
		break;
	}
	case WRITE_TEST_FILE:
	{
		UINT nb_to_write;
		UINT nb_written;

		res = f_open(&file, path, FA_WRITE | FA_CREATE_ALWAYS | FA_OPEN_ALWAYS);
		if (res == FR_OK)
		{
			debug_printf("File %s opened\n", path);
			display("Testfile opened\n");
			nb_to_write = strlen((char *)datas_to_write);
			res = f_write(&file, datas_to_write, nb_to_write, &nb_written);
			if (res == FR_OK && nb_to_write == nb_written)
				debug_printf("datas wrote in file\n");
			else
				debug_printf("Fail to write datas in file\n");
			f_close(&file);
		}
		else
		{
			debug_printf("Fail to open file %s\n", path);
		}

		state = READ_TEST_FILE;
		break;
	}
	case READ_TEST_FILE:
	{
#define READ_BUFFER_SIZE 32
		UINT nb_read = 0;
		uint8_t datas_read[READ_BUFFER_SIZE];
		res = f_open(&file, path, FA_READ);
		if (res == FR_OK)
		{
			res = f_read(&file, datas_read, READ_BUFFER_SIZE, &nb_read);
			datas_read[nb_read] = '\0'; // Pour utiliser strcmp...
			if (strcmp((char *)datas_read, (char *)datas_to_write) == 0)
				debug_printf("Correct data read\n");
			else
				debug_printf("Bad data read : %d datas : %s\n", nb_read, datas_read);
			f_close(&file);
		}
		state = DELETE_TEST_FILE;
		break;
	}
	case DELETE_TEST_FILE:
		res = f_unlink(path);
		if (res == FR_OK)
			debug_printf("Test File deleted : ok\n");
		else
			debug_printf("Error deleting test file\n");
		state = IDLE;
		break;

	case END_WITH_ERROR:
		debug_printf("SD Card : error\n");
		display("SD Card : ERROR\n");
		state = IDLE;
		break;
	case IDLE:
		if (ask_for_finish)
		{
			state = INIT;
			ret = END_OK;
		}
		break;
	default:
		break;
	}

	if (res != FR_OK)
		verbose_fresult(res);
	return ret;
}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
