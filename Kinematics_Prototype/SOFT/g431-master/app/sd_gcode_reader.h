/**
 *******************************************************************************
 * @file    sd_gcode_reader.h
 * @author  Ol, naej, Fabs
 * @date    Current Date
 * @brief   SD card G-code file reader for 3D printer/CNC control
 *******************************************************************************
 */

#ifndef SD_GCODE_READER_H
#define SD_GCODE_READER_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#if USE_SD_CARD
#include "SD/stm32g4_sd.h"
#include "SD/FatFs/src/ff.h"
#endif

/* Exported constants --------------------------------------------------------*/
#define SD_GCODE_MAX_LINE_LENGTH 128
#define SD_GCODE_MAX_FILENAME_LENGTH 64

/* Exported types ------------------------------------------------------------*/

/**
 * @brief SD G-code reader result codes
 */
typedef enum
{
    SD_GCODE_OK = 0,
    SD_GCODE_ERROR,
    SD_GCODE_NOT_READY,
    SD_GCODE_FILE_NOT_FOUND,
    SD_GCODE_READ_ERROR,
    SD_GCODE_END_OF_FILE,
    SD_GCODE_INVALID_PARAMETER
} sd_gcode_result_t;

/**
 * @brief SD G-code file information
 */
typedef struct
{
    char filename[SD_GCODE_MAX_FILENAME_LENGTH];
    uint32_t file_size;
    uint32_t lines_total;
    uint32_t lines_processed;
    uint8_t progress_percent;
    bool is_open;
} sd_gcode_file_info_t;

/**
 * @brief SD G-code reader structure
 */
typedef struct
{
#if USE_SD_CARD
    FIL file;
#endif
    sd_gcode_file_info_t info;
    uint32_t bytes_read;
    bool initialized;
    bool file_open;
} sd_gcode_reader_t;

/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief Initialize SD card G-code reader
 * @return true if successful, false otherwise
 */
bool sd_gcode_reader_init(void);

/**
 * @brief Open G-code file from SD card
 * @param filename G-code filename to open
 * @return SD_GCODE_OK if successful
 */
sd_gcode_result_t sd_gcode_reader_open(const char *filename);

/**
 * @brief Read next line from G-code file
 * @param buffer Buffer to store the line
 * @param buffer_size Size of the buffer
 * @return SD_GCODE_OK if line read successfully
 */
sd_gcode_result_t sd_gcode_reader_read_line(char *buffer, uint32_t buffer_size);

/**
 * @brief Close currently open G-code file
 */
void sd_gcode_reader_close(void);

/**
 * @brief Check if file is at end
 * @return true if at end of file
 */
bool sd_gcode_reader_is_eof(void);

/**
 * @brief Get file information
 * @return Pointer to file info structure
 */
sd_gcode_file_info_t *sd_gcode_reader_get_info(void);

/**
 * @brief Get reading progress percentage
 * @return Progress percentage (0-100)
 */
uint8_t sd_gcode_reader_get_progress(void);

/**
 * @brief List available G-code files on SD card
 * @param file_list Array to store filenames
 * @param max_files Maximum number of files to list
 * @return Number of files found
 */
uint32_t sd_gcode_reader_list_files(char file_list[][SD_GCODE_MAX_FILENAME_LENGTH], uint32_t max_files);

/**
 * @brief Check if SD card is ready
 * @return true if SD card is available
 */
bool sd_gcode_reader_is_ready(void);

#endif /* SD_GCODE_READER_H */
