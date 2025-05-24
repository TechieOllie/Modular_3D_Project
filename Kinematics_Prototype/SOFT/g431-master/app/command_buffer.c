#include "command_buffer.h"
#include "parser.h"
#include "SD/FatFs/src/ff.h" // Include the appropriate SD card file system library
#include <string.h>
#include <stdio.h>

// Static file handle for SD card operations
static FIL g_file;
static char line_buffer[128]; // Buffer for reading lines from file

// Initialize the command buffer
void cmd_buffer_init(command_buffer_t *buffer)
{
    if (buffer == NULL)
        return;

    buffer->head = 0;
    buffer->tail = 0;
    buffer->count = 0;
    buffer->status = BUFFER_EMPTY;
    buffer->error_message[0] = '\0';
    buffer->sd_file_open = false;
    buffer->current_filename[0] = '\0';
    buffer->file_position = 0;
    buffer->line_number = 0;

    // Initialize the parser
    parser_init();
}

// Open a G-code file from the SD card
buffer_status_t cmd_buffer_open_file(command_buffer_t *buffer, const char *filename)
{
    if (buffer == NULL || filename == NULL)
    {
        return BUFFER_ERROR;
    }

    // Close any existing file
    if (buffer->sd_file_open)
    {
        f_close(&g_file);
        buffer->sd_file_open = false;
    }

    // Open the new file
    FRESULT fr = f_open(&g_file, filename, FA_READ);
    if (fr != FR_OK)
    {
        sprintf(buffer->error_message, "Failed to open file: %s (error %d)", filename, fr);
        buffer->status = BUFFER_ERROR;
        return buffer->status;
    }

    // Set buffer state
    strncpy(buffer->current_filename, filename, sizeof(buffer->current_filename) - 1);
    buffer->current_filename[sizeof(buffer->current_filename) - 1] = '\0';
    buffer->sd_file_open = true;
    buffer->file_position = 0;
    buffer->line_number = 0;
    buffer->head = 0;
    buffer->tail = 0;
    buffer->count = 0;
    buffer->status = BUFFER_EMPTY;

    // Pre-fill the buffer
    return cmd_buffer_fill(buffer);
}

// Close current file
void cmd_buffer_close_file(command_buffer_t *buffer)
{
    if (buffer == NULL)
        return;

    if (buffer->sd_file_open)
    {
        f_close(&g_file);
        buffer->sd_file_open = false;
        buffer->current_filename[0] = '\0';
        buffer->status = BUFFER_EMPTY;
    }
}

// Helper function to read a line from the file
static bool read_line_from_file(FIL *fp, char *line, int max_length)
{
    int i = 0;
    UINT br;
    char c;

    while (i < max_length - 1)
    {
        // Read one character at a time
        if (f_read(fp, &c, 1, &br) != FR_OK || br == 0)
        {
            if (i == 0)
                return false; // EOF or error at beginning of line
            break;
        }

        // Check for end of line
        if (c == '\n')
        {
            break;
        }

        // Skip carriage returns
        if (c == '\r')
        {
            continue;
        }

        // Add character to line
        line[i++] = c;
    }

    // Null terminate the string
    line[i] = '\0';
    return true;
}

// Fill the buffer with commands from file
buffer_status_t cmd_buffer_fill(command_buffer_t *buffer)
{
    if (buffer == NULL)
    {
        return BUFFER_ERROR;
    }

    if (!buffer->sd_file_open)
    {
        sprintf(buffer->error_message, "No file open");
        buffer->status = BUFFER_ERROR;
        return buffer->status;
    }

    // Fill buffer until full or EOF
    while (buffer->count < CMD_BUFFER_SIZE)
    {
        // If we've reached EOF, break
        if (!read_line_from_file(&g_file, line_buffer, sizeof(line_buffer)))
        {
            buffer->status = buffer->count > 0 ? BUFFER_READY : BUFFER_EMPTY;
            return buffer->status;
        }

        buffer->line_number++;
        buffer->file_position = f_tell(&g_file);

        // Skip empty lines and comments
        if (line_buffer[0] == '\0' || line_buffer[0] == ';' ||
            line_buffer[0] == '(' || line_buffer[0] == '%')
        {
            continue;
        }

        // Parse the line
        parser_command_t *cmd = &buffer->commands[buffer->head];
        if (!parser_parse_line(line_buffer, cmd))
        {
            // If parsing fails, store the error and continue
            // In a production system, you might want to handle this differently
            sprintf(buffer->error_message, "Parse error at line %ld: %s",
                    (long)buffer->line_number, parser_get_error());
            continue;
        }

        // Advance head pointer
        buffer->head = (buffer->head + 1) % CMD_BUFFER_SIZE;
        buffer->count++;
    }

    buffer->status = BUFFER_FULL;
    return buffer->status;
}

// Check if buffer has a command
bool cmd_buffer_has_command(command_buffer_t *buffer)
{
    if (buffer == NULL)
        return false;
    return buffer->count > 0;
}

// Get the next command without removing it
parser_command_t *cmd_buffer_get_command(command_buffer_t *buffer)
{
    if (buffer == NULL || buffer->count == 0)
    {
        return NULL;
    }

    return &buffer->commands[buffer->tail];
}

// Remove the current command from the buffer after executing it
void cmd_buffer_consume_command(command_buffer_t *buffer)
{
    if (buffer == NULL || buffer->count == 0)
    {
        return;
    }

    // Move tail pointer to next command
    buffer->tail = (buffer->tail + 1) % CMD_BUFFER_SIZE;
    buffer->count--;

    // If buffer is getting low, try to fill it
    if (buffer->count <= CMD_BUFFER_SIZE / 4)
    {
        cmd_buffer_fill(buffer);
    }
}

// Get error message
const char *cmd_buffer_get_error(command_buffer_t *buffer)
{
    if (buffer == NULL)
        return "Invalid buffer";
    return buffer->error_message;
}

// Get current line number
uint32_t cmd_buffer_get_line_number(command_buffer_t *buffer)
{
    if (buffer == NULL)
        return 0;
    return buffer->line_number;
}

// Calculate progress percentage
float cmd_buffer_get_progress(command_buffer_t *buffer)
{
    if (buffer == NULL || !buffer->sd_file_open)
        return 0.0f;

    DWORD file_size;
    if (f_size(&g_file) == 0)
        return 0.0f;

    file_size = f_size(&g_file);
    return (float)buffer->file_position * 100.0f / (float)file_size;
}

// Check if we've reached the end of file
bool cmd_buffer_is_eof(command_buffer_t *buffer)
{
    if (buffer == NULL || !buffer->sd_file_open)
        return true;

    DWORD file_size = f_size(&g_file);
    return buffer->file_position >= file_size && buffer->count == 0;
}