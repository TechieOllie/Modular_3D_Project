/**
 *******************************************************************************
 * @file    sd_gcode_reader.c
 * @author  Ol, naej, Fabs
 * @date    Current Date
 * @brief   SD card G-code file reader implementation
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "sd_gcode_reader.h"
#include "config.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#if USE_SD_CARD
#include "SD/stm32g4_sd.h"
#include "SD/FatFs/src/ff.h"

/* Add missing string comparison function */
static int my_strcasecmp(const char *s1, const char *s2)
{
    while (*s1 && *s2)
    {
        char c1 = tolower(*s1);
        char c2 = tolower(*s2);
        if (c1 != c2)
            return c1 - c2;
        s1++;
        s2++;
    }
    return tolower(*s1) - tolower(*s2);
}

#endif /* USE_SD_CARD */

#if USE_SD_CARD

/* Private variables ---------------------------------------------------------*/
static sd_gcode_reader_t reader = {0};
static FIL gcode_file;
static FATFS fatfs;
static bool sd_mounted = false;

/* Private function prototypes -----------------------------------------------*/
static bool mount_sd_card(void);
static void unmount_sd_card(void);
static bool is_gcode_file(const char *filename);

/* Public function implementations -------------------------------------------*/

/**
 * @brief Initialize SD card G-code reader
 */
bool sd_gcode_reader_init(void)
{
    printf("Initializing SD card G-code reader...\n");

    // Initialize SD card hardware
    if (BSP_SD_Init() != BSP_SD_OK)
    {
        printf("ERROR: Failed to initialize SD card hardware\n");
        return false;
    }

    // Mount filesystem
    if (!mount_sd_card())
    {
        printf("ERROR: Failed to mount SD card filesystem\n");
        return false;
    }

    // Initialize reader structure
    memset(&reader, 0, sizeof(reader));
    reader.initialized = true;
    reader.file_open = false;

    printf("SD card G-code reader initialized successfully\n");
    return true;
}

/**
 * @brief Open G-code file from SD card
 */
sd_gcode_result_t sd_gcode_reader_open(const char *filename)
{
    if (!reader.initialized)
    {
        return SD_GCODE_NOT_READY;
    }

    if (!filename || strlen(filename) == 0)
    {
        return SD_GCODE_INVALID_PARAMETER;
    }

    // Close any currently open file
    if (reader.file_open)
    {
        sd_gcode_reader_close();
    }
    // Open file
    FRESULT res = f_open(&gcode_file, filename, FA_READ);
    if (res != FR_OK)
    {
        printf("ERROR: Failed to open G-code file '%s' (error %d)\n", filename, res);
        return SD_GCODE_FILE_NOT_FOUND;
    }

    // Initialize file info
    // Initialize file info
    strncpy(reader.info.filename, filename, SD_GCODE_MAX_FILENAME_LENGTH - 1);
    reader.info.file_size = f_size(&gcode_file);
    reader.info.lines_total = 0; // Will be estimated during reading
    reader.info.progress_percent = 0;
    reader.info.is_open = true;
    reader.bytes_read = 0;
    reader.file_open = true;

    printf("G-code file '%s' opened (size: %lu bytes)\n", filename, reader.info.file_size);
    return SD_GCODE_OK;
}

/**
 * @brief Read next line from G-code file
 */
sd_gcode_result_t sd_gcode_reader_read_line(char *buffer, uint32_t buffer_size)
{
    if (!reader.initialized || !reader.file_open)
    {
        return SD_GCODE_NOT_READY;
    }

    if (!buffer || buffer_size == 0)
    {
        return SD_GCODE_INVALID_PARAMETER;
    }

    // Read line from file
    char *result = f_gets(buffer, buffer_size, &gcode_file);
    if (result == NULL)
    {
        if (f_eof(&gcode_file))
        {
            return SD_GCODE_END_OF_FILE;
        }
        return SD_GCODE_READ_ERROR;
    }

    // Update progress
    reader.bytes_read = f_tell(&gcode_file);
    reader.info.lines_processed++;

    if (reader.info.file_size > 0)
    {
        reader.info.progress_percent = (uint8_t)((reader.bytes_read * 100) / reader.info.file_size);
    }

    // Remove newline characters
    size_t len = strlen(buffer);
    while (len > 0 && (buffer[len - 1] == '\n' || buffer[len - 1] == '\r'))
    {
        buffer[--len] = '\0';
    }

    return SD_GCODE_OK;
}

/**
 * @brief Close currently open G-code file
 */
void sd_gcode_reader_close(void)
{
    if (reader.file_open)
    {
        f_close(&gcode_file);
        reader.file_open = false;
        reader.info.is_open = false;
        printf("G-code file closed\n");
    }
}

/**
 * @brief Check if end of file is reached
 */
bool sd_gcode_reader_is_eof(void)
{
    if (!reader.file_open)
    {
        return true;
    }
    return f_eof(&gcode_file);
}
/**
 * @brief Get file information
 */
sd_gcode_file_info_t *sd_gcode_reader_get_info(void)
{
    return &reader.info;
}

/**
 * @brief Get reading progress percentage
 */
uint8_t sd_gcode_reader_get_progress(void)
{
    return reader.info.progress_percent;
}

/**
 * @brief List available G-code files on SD card
 */
uint32_t sd_gcode_reader_list_files(char file_list[][SD_GCODE_MAX_FILENAME_LENGTH], uint32_t max_files)
{
    if (!reader.initialized || !file_list || max_files == 0)
    {
        return 0;
    }

    DIR dir;
    FILINFO fno;
    uint32_t file_count = 0;

    // Open root directory
    FRESULT res = f_opendir(&dir, "/");
    if (res != FR_OK)
    {
        printf("ERROR: Failed to open root directory\n");
        return 0;
    }

    // Read directory entries
    while (file_count < max_files)
    {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0)
        {
            break; // End of directory or error
        }

        // Skip directories and non-G-code files
        if (fno.fattrib & AM_DIR)
        {
            continue;
        }

        if (is_gcode_file(fno.fname))
        {
            strncpy(file_list[file_count], fno.fname, SD_GCODE_MAX_FILENAME_LENGTH - 1);
            file_list[file_count][SD_GCODE_MAX_FILENAME_LENGTH - 1] = '\0';
            file_count++;
        }
    }

    f_closedir(&dir);

    printf("Found %lu G-code files on SD card\n", file_count);
    return file_count;
}

/**
 * @brief Check if SD card is ready
 */
bool sd_gcode_reader_is_ready(void)
{
    return reader.initialized && sd_mounted;
}

/* Private function implementations ------------------------------------------*/

/**
 * @brief Mount SD card filesystem
 */
static bool mount_sd_card(void)
{
    FRESULT res = f_mount(&fatfs, "", 1);
    if (res != FR_OK)
    {
        printf("ERROR: Failed to mount SD card (error %d)\n", res);
        return false;
    }

    sd_mounted = true;
    printf("SD card mounted successfully\n");
    return true;
}

/**
 * @brief Unmount SD card filesystem
 */
static void unmount_sd_card(void)
{
    if (sd_mounted)
    {
        f_mount(NULL, "", 0);
        sd_mounted = false;
        printf("SD card unmounted\n");
    }
}

/**
 * @brief Check if filename is a G-code file
 */
static bool is_gcode_file(const char *filename)
{
    if (!filename)
    {
        return false;
    }

    size_t len = strlen(filename);
    if (len < 3)
    {
        return false;
    }

    // Check for common G-code file extensions
    const char *ext = &filename[len - 3];
    if (len >= 3 && (my_strcasecmp(ext, ".gc") == 0 || my_strcasecmp(ext, ".nc") == 0))
    {
        return true;
    }

    if (len >= 4)
    {
        ext = &filename[len - 4];
        if (my_strcasecmp(ext, ".gco") == 0 || my_strcasecmp(ext, ".gcode") == 0)
        {
            return true;
        }
    }

    if (len >= 5)
    {
        ext = &filename[len - 5];
        if (my_strcasecmp(ext, ".gcode") == 0)
        {
            return true;
        }
    }

    return false;
}

#else /* !USE_SD_CARD */

/* Stub implementations when SD card is disabled */
bool sd_gcode_reader_init(void) { return false; }
sd_gcode_result_t sd_gcode_reader_open(const char *filename) { return SD_GCODE_NOT_READY; }
sd_gcode_result_t sd_gcode_reader_read_line(char *buffer, uint32_t buffer_size) { return SD_GCODE_NOT_READY; }
void sd_gcode_reader_close(void) {}
bool sd_gcode_reader_is_eof(void) { return true; }
sd_gcode_file_info_t *sd_gcode_reader_get_info(void) { return NULL; }
uint8_t sd_gcode_reader_get_progress(void) { return 0; }
uint32_t sd_gcode_reader_list_files(char file_list[][SD_GCODE_MAX_FILENAME_LENGTH], uint32_t max_files) { return 0; }
bool sd_gcode_reader_is_ready(void) { return false; }

#endif /* USE_SD_CARD */
