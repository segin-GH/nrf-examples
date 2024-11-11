#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/sys/crc.h>
#include <zephyr/sys/printk.h>

#include "frame_format.h"
static ff_err_t ff_unpack_frame(frame_frm_t *frame, const uint8_t *filter_data)
{
    if (!frame || !filter_data)
    {
        printk("Invalid input\n");
        return FF_ERR_INVALID_INPUT;
    }

    frame->soh = filter_data[0];                         // 1 byte
    frame->ver = filter_data[1];                         // 1 byte
    frame->cmd = (filter_data[2] << 8) | filter_data[3]; // 2 bytes
    frame->ssoh = filter_data[4];                        // 1 byte
    frame->len = (filter_data[5] << 8) | filter_data[6]; // 2 bytes

    // Allocate memory for buff and copy data
    frame->buff = malloc(frame->len * sizeof(uint8_t));
    if (!frame->buff)
    {
        printk("Memory allocation failed\n");
        return FF_ERR_NO_MEMORY;
    }

    memcpy(frame->buff, &filter_data[7], frame->len);
    frame->id = filter_data[7 + frame->len];

    frame->crc = (filter_data[8 + frame->len] << 24) | (filter_data[9 + frame->len] << 16) |
                 (filter_data[10 + frame->len] << 8) | filter_data[11 + frame->len];

    frame->eot = filter_data[12 + frame->len];

    return FF_ERR_OK;
}

static void ff_print_uint8_array(const char *name, uint8_t *data, size_t len)
{
    printk("%s: ", name);
    for (size_t i = 0; i < len; i++)
    {
        printk("%02x", data[i]);
    }
    printk("\n");
}

// Function to add escape sequence to data, excluding first and last bytes (header/footer)
uint8_t *ff_add_escape_sequence(const uint8_t *data, size_t len, size_t *new_len)
{
    if (len <= 2)
    {
        // If there are only header and footer bytes, no need to escape anything
        *new_len = len;
        uint8_t *result = (uint8_t *)malloc(len);
        if (result != NULL)
        {
            for (size_t i = 0; i < len; i++)
            {
                result[i] = data[i]; // Just copy the data
            }
        }
        return result;
    }

    // Step 1: Calculate new length (excluding the first and last bytes)
    size_t escaped_len = len; // Initial length includes header and footer without modification

    for (size_t i = 1; i < len - 1; i++)
    { // Start from index 1 and end at len - 1 (exclude header/footer)
        uint8_t byte = data[i];
        if (byte == FF_SOF_CHAR || byte == FF_EOF_CHAR || byte == FF_ESC_CHAR)
        {
            escaped_len += 1; // Special byte requires escape + XORed byte, adding an extra byte
        }
    }

    // Step 2: Allocate memory for the new buffer
    uint8_t *escaped_data = (uint8_t *)malloc(escaped_len);
    if (escaped_data == NULL)
    {
        // Handle memory allocation failure
        *new_len = 0;
        return NULL;
    }

    // Step 3: Copy the header byte as-is
    escaped_data[0] = data[0]; // Header (first byte)

    // Step 4: Process and escape the middle bytes (excluding header and footer)
    size_t idx = 1;
    for (size_t i = 1; i < len - 1; i++)
    {
        uint8_t byte = data[i];
        if (byte == FF_SOF_CHAR || byte == FF_EOF_CHAR || byte == FF_ESC_CHAR)
        {
            escaped_data[idx++] = FF_ESC_CHAR;        // Insert escape character
            escaped_data[idx++] = byte ^ FF_XOR_MASK; // Insert XORed byte
        }
        else
        {
            escaped_data[idx++] = byte; // Insert regular byte
        }
    }

    // Step 5: Copy the footer byte as-is
    escaped_data[idx] = data[len - 1]; // Footer (last byte)
    ff_print_uint8_array("Escaped data", escaped_data, escaped_len);

    *new_len = escaped_len; // Update new length
    return escaped_data;
}

static size_t ff_calculate_filtered_length(const uint8_t *data, size_t data_len)
{
    size_t i = 0, filtered_len = 0;

    while (i < data_len)
    {
        if (data[i] == FF_ESC_CHAR)
        {
            i += 2; // Skip the escape character and the following XOR-ed byte
        }
        else
        {
            i++;
        }
        filtered_len++;
    }

    return filtered_len;
}

static size_t ff_rm_escape_sequnce(uint8_t *raw_buff, size_t raw_len, uint8_t *filter_data)
{
    size_t i = 0, j = 0;

    while (i < raw_len)
    {
        if (raw_buff[i] == FF_ESC_CHAR)
        {
            i++;                                            // Skip the escape character
            filter_data[j++] = (raw_buff[i] ^ FF_XOR_MASK); // Apply XOR mask and store result
            i++;
        }
        else
        {
            filter_data[j++] = raw_buff[i++]; // Copy non-escaped byte
        }
    }

    return j;
}

static ff_err_t ff_validate_raw_frame(uint8_t *raw_buff, uint16_t len)
{
    if (!raw_buff)
    {
        printk("Invalid input\n");
        return FF_ERR_INVALID_INPUT;
    }

    if (len < 0 || len > 65535)
    {
        printk("Invalid buffer length\n");
        return FF_ERR_INVALID_INPUT;
    }

    if (raw_buff[0] != FF_SOF_CHAR)
    {
        printk("Invalid SOF\n");
        return FF_ERR_SOF_MISMATCH;
    }

    if (raw_buff[len - 1] != FF_EOF_CHAR)
    {
        printk("Invalid EOF\n");
        return FF_ERR_EOF_MISMATCH;
    }

    /* Validate the CRC */
    uint32_t crc = (raw_buff[len - 5] << 24) | (raw_buff[len - 4] << 16) | (raw_buff[len - 3] << 8) | raw_buff[len - 2];
    printk("CRC: %08x\n", crc);

    uint32_t crc_calc = crc32_ieee(raw_buff, len - 5);
    printk("CRC Calc: %08x\n", crc_calc);

    if (crc != crc_calc)
    {
        printk("CRC mismatch\n");
        return FF_ERR_CRC_MISMATCH;
    }

    return FF_ERR_OK;
}

size_t ff_get_rawbuff_len(frame_frm_t *frame)
{

    return (13 + frame->len);
}

uint8_t *ff_pack_frame(frame_frm_t *frame)
{
    if (!frame)
    {
        printk("Invalid input\n");
        return NULL;
    }

    uint8_t *packed_data = malloc(ff_get_rawbuff_len(frame));
    if (!packed_data)
    {
        printk("Memory allocation failed\n");
        return NULL;
    }

    // Pack the frame fields
    packed_data[0] = frame->soh;
    packed_data[1] = frame->ver;
    packed_data[2] = (frame->cmd >> 8) & 0xFF;
    packed_data[3] = frame->cmd & 0xFF;
    packed_data[4] = frame->ssoh;
    packed_data[5] = (frame->len >> 8) & 0xFF;
    packed_data[6] = frame->len & 0xFF;

    // Copy buffer data
    memcpy(&packed_data[7], frame->buff, frame->len);

    // Add frame ID
    packed_data[7 + frame->len] = frame->id;

    // Add CRC
    packed_data[8 + frame->len] = (frame->crc >> 24) & 0xFF;
    packed_data[9 + frame->len] = (frame->crc >> 16) & 0xFF;
    packed_data[10 + frame->len] = (frame->crc >> 8) & 0xFF;
    packed_data[11 + frame->len] = frame->crc & 0xFF;

    // Add EOT
    packed_data[12 + frame->len] = frame->eot;

    return packed_data;
}

ff_err_t ff_process_frame(frame_frm_t *frame, uint8_t *raw_buff, uint16_t len)
{
    if (!frame)
    {
        printk("Invalid input\n");
        return FF_ERR_INVALID_INPUT;
    }

    if (!raw_buff)
    {
        printk("Invalid buffer\n");
        return FF_ERR_INVALID_INPUT;
    }

    if (len < 0 || len > 65535)
    {
        printk("Invalid buffer length\n");
        return FF_ERR_INVALID_INPUT;
    }

    ff_err_t ret = FF_ERR_OK;

    /* Print the raw data */
    // ff_print_uint8_array("R data", raw_buff, len);

    /* Remove escape characters from the raw buffer */
    size_t cal_filtered_len = ff_calculate_filtered_length(raw_buff, len);
    // printk("Filtered length: %zu\n", cal_filtered_len);

    uint8_t *filter_data = malloc(sizeof(uint8_t) * cal_filtered_len);
    if (!filter_data)
    {
        printk("Memory allocation failed\n");
        return FF_ERR_INVALID_INPUT;
    }

    size_t actual_filtered_len = ff_rm_escape_sequnce(raw_buff, len, filter_data);

    if (cal_filtered_len != actual_filtered_len)
    {
        printk("Error in removing escape characters got %zu, expected %zu\n", actual_filtered_len, cal_filtered_len);
        return FF_ERR_INVALID_INPUT;
    }

    /* Validate the raw frame */
    ret = ff_validate_raw_frame(filter_data, actual_filtered_len);
    if (ret != FF_ERR_OK)
    {
        printk("Error in validating raw frame\n");
        return ret;
    }

    /* Process the frame */
    ret = ff_unpack_frame(frame, filter_data);
    if (ret != FF_ERR_OK)
    {
        printk("Error in unpacking frame\n");
        return ret;
    }

    free(filter_data);
    return FF_ERR_OK;
}

ff_buff_t *ff_create_frame(frame_frm_t *frame)
{

    if (!frame)
    {
        printk("Invalid input Args is NULL\n");
        return NULL;
    }

    uint8_t *buffer = ff_pack_frame(frame);
    if (!buffer)
    {
        printk("Error in packing frame\n");
        return NULL;
    }

    /* Add escape sequence */
    int new_len;

    uint8_t *buff = ff_add_escape_sequence(buffer, ff_get_rawbuff_len(frame), &new_len);
    if (!buff)
    {
        printk("Error in adding escape sequence\n");
        return NULL;
    }

    free(buffer);

    ff_buff_t *ff_buff = malloc(sizeof(ff_buff_t));
    if (!ff_buff)
    {
        printk("Memory allocation failed\n");
        return NULL;
    }

    ff_buff->buff = buff;
    ff_buff->len = new_len;

    return ff_buff;
}

void ff_free_frame(ff_buff_t *ff_buff)
{
    if (ff_buff)
    {
        if (ff_buff->buff)
        {
            free(ff_buff->buff);
        }
        free(ff_buff);
    }
}
