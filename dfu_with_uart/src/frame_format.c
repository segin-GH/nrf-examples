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

    frame->soh  = filter_data[0];                         // 1 byte
    frame->ver  = filter_data[1];                         // 1 byte
    frame->cmd  = (filter_data[2] << 8) | filter_data[3]; // 2 bytes
    frame->ssoh = filter_data[4];                         // 1 byte
    frame->len  = (filter_data[5] << 8) | filter_data[6]; // 2 bytes

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
