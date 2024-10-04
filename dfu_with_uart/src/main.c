#include "frame_format.h"
#include "zephyr/sys/printk.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/dfu/flash_img.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/reboot.h>

#define LED_NODE DT_ALIAS(led1)
#define FW_CHUNK_SIZE 512

#define ESC_CHAR 0x7D
#define SOF_CHAR 0xAA
#define EOF_CHAR 0x55
#define XOR_MASK 0x20

#define MCUBOOT_PARTITION_ID FLASH_AREA_ID(mcuboot)
#define IMAGE_0_PARTITION_ID FLASH_AREA_ID(image_0)
#define IMAGE_1_PARTITION_ID FLASH_AREA_ID(image_1)
#define STORAGE_PARTITION_ID FLASH_AREA_ID(storage)

#define CURRENT_IMAGE_PARTITION_ID IMAGE_1_PARTITION_ID

#define UART_QUEUE_MAX_SIZE 10
#define UART_MAX_RX 1024

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

typedef struct
{
    uint8_t soh;
    uint8_t ver;
    uint16_t len;
    uint16_t cmd;
    void *buff;
    uint32_t crc;
    uint8_t eot;
} frame_fmt_t;

/* Define a struct to hold the buffer and its length */
struct uart_data
{
    char data[UART_MAX_RX];
    size_t len;
};

/* Define the message queue to hold uart_data structs */
K_MSGQ_DEFINE(uart_msgq, sizeof(struct uart_data), UART_QUEUE_MAX_SIZE, 4);

/* Buffer to temporarily hold incoming data */
static struct uart_data rx_buf;
static int rx_buf_pos;

void serial_cb(const struct device *dev, void *user_data)
{
    uint8_t c;
    if (!uart_irq_update(dev))
    {
        return;
    }

    /* read until FIFO empty */
    while (uart_fifo_read(dev, &c, 1) == 1)
    {
        if ((c == 0x55) && rx_buf_pos > 0)
        {
            /* terminate buffer with the special character */
            rx_buf.data[rx_buf_pos] = c;

            /* set the length of the data in the struct */
            rx_buf.len = rx_buf_pos + 1;

            /* if queue is full, message is silently dropped */
            k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

            /* reset the buffer (it was copied to the msgq) */
            rx_buf_pos = 0;
        }
        else if (rx_buf_pos < (sizeof(rx_buf.data) - 1))
        {
            rx_buf.data[rx_buf_pos++] = c;
            // FIXME: if buffer is full, we should drop the message?
        }
    }
}

int init_uart(const struct device *dev)
{
    if (!device_is_ready(dev))
    {
        printk("UART device not found!");
        return 0;
    }

    /* configure interrupt and callback to receive data */
    int ret = uart_irq_callback_user_data_set(dev, serial_cb, NULL);
    if (ret < 0)
    {
        if (ret == -ENOTSUP)
        {
            printk("Interrupt-driven UART API support not enabled\n");
        }
        else if (ret == -ENOSYS)
        {
            printk("UART device does not support interrupt-driven API\n");
        }
        else
        {
            printk("Error setting UART callback: %d\n", ret);
        }
        return 0;
    }

    uart_irq_rx_enable(dev);
    return 0;
}

void print_partition_info(int partition_id, const char *label)
{
    const struct flash_area *fa;

    int err = flash_area_open(partition_id, &fa);
    if (err)
    {
        printk("Failed to open partition: %s (ID: %d)\n", label, partition_id);
        return;
    }

    printk("Partition: %s\n", label);
    printk("  Offset: 0x%lx\n", (unsigned long)fa->fa_off);
    printk("  Size: 0x%lx\n", (unsigned long)fa->fa_size);

    flash_area_close(fa);
}

typedef enum
{
    STATE_IDLE,
    STATE_DFU_WRITING,
    STATE_DFU_VALIDATE,
    STATE_DFU_REBOOT,
    STATE_ERROR
} dfu_state_t;

typedef enum
{
    DFU_ERR_FRAME_FORMAT = 1,
    DFU_ERR_FLASH_WRITE,
    DFU_ERR_FLASH_VALIDATE,
    DFU_ERR_FLASH_REBOOT
} dfu_err_t;

typedef struct
{
    uint8_t *buffer;
    size_t buffer_len;
    bool is_valid;
    int error;
} state_dfu_context_t;

dfu_state_t state = STATE_IDLE;

// Function prototypes
void handle_idle(state_dfu_context_t *state_ctx, struct flash_img_context *ctx, const struct device *uart_dev);
void handle_dfu_writing(state_dfu_context_t *state_ctx, struct flash_img_context *ctx, const struct device *uart_dev);
void handle_dfu_validate(state_dfu_context_t *state_ctx, struct flash_img_context *ctx, const struct device *uart_dev);
void handle_dfu_reboot(state_dfu_context_t *state_ctx);
void handle_error(state_dfu_context_t *state_ctx, struct flash_img_context *ctx, const struct device *uart_dev);

int main(void)
{
    printk("Hello World! 2 State machine OTA update\n");

    // LED initialization
    int err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

    // Print partition info
    print_partition_info(MCUBOOT_PARTITION_ID, "mcuboot");
    print_partition_info(IMAGE_0_PARTITION_ID, "image-0");
    print_partition_info(IMAGE_1_PARTITION_ID, "image-1");
    print_partition_info(STORAGE_PARTITION_ID, "storage");

    // Initialize flash image context
    struct flash_img_context ctx = {0};
    err = flash_img_init_id(&ctx, CURRENT_IMAGE_PARTITION_ID);
    if (err)
    {
        printk("Flash img failed\n");
        k_sleep(K_MSEC(1000));
        return -1;
    }

    printk("Flash img initialized\n");
    printk("Flash size: %d\n", ctx.flash_area->fa_size);
    printk("Flash offset: %ld\n", ctx.flash_area->fa_off);

    // Initialize UART
    const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(dozee_sheet_smallsection));
    init_uart(uart_dev);

    // State context
    state_dfu_context_t state_ctx = {0};

    // State machine loop
    while (1)
    {
        switch (state)
        {
        case STATE_IDLE:
            handle_idle(&state_ctx, &ctx, uart_dev);
            state = STATE_DFU_WRITING;
            break;

        case STATE_DFU_WRITING:
            handle_dfu_writing(&state_ctx, &ctx, uart_dev);
            state = STATE_DFU_VALIDATE;
            break;

        case STATE_DFU_VALIDATE:
            handle_dfu_validate(&state_ctx, &ctx, uart_dev);
            state = STATE_DFU_REBOOT;
            break;

        case STATE_DFU_REBOOT:
            handle_dfu_reboot(&state_ctx);
            state = STATE_IDLE;
            break;

        case STATE_ERROR:
            handle_error(&state_ctx, &ctx, uart_dev);
            state = STATE_IDLE;
            break;

        default:
            printk("Invalid state, transitioning to STATE_ERROR\n");
            state = STATE_ERROR;
            break;
        }
    }

    return 0;
}

void handle_idle(state_dfu_context_t *state_ctx, struct flash_img_context *ctx, const struct device *uart_dev)
{
    printk("STATE_IDLE\n");

    while (k_msgq_get(&uart_msgq, &rx_buf, K_FOREVER) == 0)
    {
        frame_frm_t frame;
        ff_err_t ff_err = ff_process_frame(&frame, rx_buf.data, rx_buf.len);
        if (ff_err != FF_ERR_OK)
        {
            printk("Error processing frame: %d\n", ff_err);
            state_ctx->error = DFU_ERR_FRAME_FORMAT;
            state = STATE_ERROR;
            break;
        }

        if (frame.cmd == 0xF1)
        {
            printk("Received start command\n");
            // Erase the flash area
            int ret = flash_area_erase(ctx->flash_area, 0, ctx->flash_area->fa_size);
            if (ret != 0)
            {
                printk("Flash erase failed (%d)\n", ret);
                state_ctx->error = DFU_ERR_FLASH_WRITE;
                state = STATE_ERROR;
                break;
            }

            printk("Flash erased\n");

            if (!frame.buff && frame.len != 2)
            {
                printk("Invalid length for start command\n");
                state_ctx->error = DFU_ERR_FRAME_FORMAT;
                state = STATE_ERROR;
                break;
            }

            uint16_t total_chunks = ((uint8_t *)frame.buff)[0] << 8 | ((uint8_t *)frame.buff)[1];
            printk("Total chunks: %d\n", total_chunks);

            state_ctx->buffer = frame.buff;
            state_ctx->buffer_len = frame.len;
            state_ctx->is_valid = true;

            frame_frm_t frmt = FF_FRAME_POPULATE(0xF2, FF_SSOH_1, 0, NULL);
        }
    }
}

void handle_dfu_writing(state_dfu_context_t *state_ctx, struct flash_img_context *ctx, const struct device *uart_dev)
{
    printk("STATE_DFU_WRITING\n");

    struct uart_data rx_buf = {0};
    int i = 0;
    int total_chunks_to_write = 0;

    if (state_ctx->is_valid)
    {
        total_chunks_to_write = state_ctx->buffer[0] << 8 | state_ctx->buffer[1];
        printk("Total chunks to write: %d\n", total_chunks_to_write);
        free(state_ctx->buffer);
    }

    while (k_msgq_get(&uart_msgq, &rx_buf, K_FOREVER) == 0)
    {
        frame_frm_t frame;
        ff_err_t ff_err = ff_process_frame(&frame, rx_buf.data, rx_buf.len);
        if (ff_err != FF_ERR_OK)
        {
            printk("Error processing frame: %d\n", ff_err);
            state_ctx->error = DFU_ERR_FRAME_FORMAT;
            state = STATE_ERROR;
            break;
        }

        printk("Data length: %d\n", frame.len);

        if (frame.buff && frame.len == 512)
        {
            int err = flash_img_buffered_write(ctx, frame.buff, frame.len, false);
            if (err)
            {
                printk("Failed to write to flash: %d\n", err);
                state_ctx->error = DFU_ERR_FLASH_WRITE;
                state = STATE_ERROR;
                break;
            }
            printk(" (%i) Wrote %d bytes:\n", i, frame.len);
        }
        else
        {
            int err = flash_img_buffered_write(ctx, frame.buff, frame.len, true);
            if (err)
            {
                printk("Failed to write to flash: %d\n", err);
                state_ctx->error = DFU_ERR_FLASH_WRITE;
                state = STATE_ERROR;
                break;
            }

            printk(" (%i) Wrote %d bytes final:\n", i, frame.len);
            printk(" Done writing\n");
            free(frame.buff);
            break;
        }

        free(frame.buff);
        memset(&rx_buf, 0, sizeof(rx_buf));
        i++;
    }
}

void handle_dfu_validate(state_dfu_context_t *state_ctx, struct flash_img_context *ctx, const struct device *uart_dev)
{
    printk("STATE_DFU_VALIDATE\n");
    // Get the total size of data written to the flash image
    size_t total_size = flash_img_bytes_written(ctx);
    printk("Total size: %d\n", total_size);

    // Clear the receive buffer
    memset(&rx_buf, 0, sizeof(rx_buf));

    // Process incoming messages from UART
    while (k_msgq_get(&uart_msgq, &rx_buf, K_FOREVER) == 0)
    {
        frame_frm_t frame;
        ff_err_t ff_err = ff_process_frame(&frame, rx_buf.data, rx_buf.len);
        if (ff_err != FF_ERR_OK)
        {
            printk("Error processing frame: %d\n", ff_err);
            state_ctx->error = DFU_ERR_FRAME_FORMAT;
            state = STATE_ERROR;
            break;
        }

        // Check for the correct command (0xF3)
        if (frame.cmd != 0xF3)
        {
            printk("Invalid command: %d\n", frame.cmd);
            state_ctx->error = DFU_ERR_FRAME_FORMAT;
            state = STATE_ERROR;
            break;
        }

        if (!frame.buff && frame.len != 32)
        {
            printk("Invalid SHA256 hash length: %d\n", frame.len);
            state_ctx->error = DFU_ERR_FRAME_FORMAT;
            state = STATE_ERROR;
            break;
        }

        const struct flash_img_check check = {
            .match = (uint8_t *)frame.buff,
            .clen = total_size,
        };

        int hash_match = flash_img_check(ctx, &check, IMAGE_1_PARTITION_ID);
        if (hash_match == 0)
        {
            printk("Hash match\n");
            state = STATE_DFU_REBOOT;
            break;
        }
        else
        {
            printk("Hash invalid\n");
            state_ctx->error = DFU_ERR_FLASH_VALIDATE;
            state = STATE_ERROR;
            break;
        }

        free(frame.buff);
    }
}

void handle_dfu_reboot(state_dfu_context_t *state_ctx)
{
    printk("STATE_DFU_REBOOT\n");

    // Request the upgrade
    int err = boot_request_upgrade_multi(IMAGE_1_PARTITION_ID, BOOT_UPGRADE_PERMANENT);
    if (err)
    {
        printk("Failed to request upgrade: %d\n", err);
        state = STATE_ERROR;
        return;
    }

    k_sleep(K_MSEC(5000));
    sys_reboot(SYS_REBOOT_COLD);
}

void handle_error(state_dfu_context_t *state_ctx, struct flash_img_context *ctx, const struct device *uart_dev)
{
    printk("STATE_ERROR\n");

    switch (state_ctx->error)
    {
    case DFU_ERR_FRAME_FORMAT:
        printk("Error: Frame format\n");
        break;

    case DFU_ERR_FLASH_WRITE:
        printk("Error: Flash write\n");
        break;

    case DFU_ERR_FLASH_VALIDATE:
        printk("Error: Flash validate\n");
        break;

    case DFU_ERR_FLASH_REBOOT:
        printk("Error: Flash reboot\n");
        break;

    default:
        printk("Unknown error\n");
        break;
    }

    k_sleep(K_MSEC(1000));
}
