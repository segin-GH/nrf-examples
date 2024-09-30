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

#define LED_NODE      DT_ALIAS(led1)
#define FW_CHUNK_SIZE 512

#define ESC_CHAR 0x7D
#define SOF_CHAR 0xAA
#define EOF_CHAR 0x55
#define XOR_MASK 0x20

#define MCUBOOT_PARTITION_ID FLASH_AREA_ID(mcuboot)
#define IMAGE_0_PARTITION_ID FLASH_AREA_ID(image_0)
#define IMAGE_1_PARTITION_ID FLASH_AREA_ID(image_1)
#define STORAGE_PARTITION_ID FLASH_AREA_ID(storage)

#define UART_QUEUE_MAX_SIZE 10
#define UART_MAX_RX         1024

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

typedef struct
{
    uint8_t  soh;
    uint8_t  ver;
    uint16_t len;
    uint16_t cmd;
    void    *buff;
    uint32_t crc;
    uint8_t  eot;
} frame_fmt_t;

/* Define a struct to hold the buffer and its length */
struct uart_data
{
    char   data[UART_MAX_RX];
    size_t len;
};

/* Define the message queue to hold uart_data structs */
K_MSGQ_DEFINE(uart_msgq, sizeof(struct uart_data), UART_QUEUE_MAX_SIZE, 4);

/* Buffer to temporarily hold incoming data */
static struct uart_data rx_buf;
static int              rx_buf_pos;

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

int main(void)
{
    printk("Hello World! 9\n");
    int err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

    print_partition_info(MCUBOOT_PARTITION_ID, "mcuboot");
    print_partition_info(IMAGE_0_PARTITION_ID, "image-0");
    print_partition_info(IMAGE_1_PARTITION_ID, "image-1");
    print_partition_info(STORAGE_PARTITION_ID, "storage");

    struct flash_img_context ctx = {0};
    err                          = flash_img_init(&ctx);
    if (err)
    {
        printk("Flash img failed\n");
        return -1;
    }

    printk("Flash img initialized\n");
    printk("Flash size: %d\n", ctx.flash_area->fa_size);
    printk("Flash offset: %ld\n", ctx.flash_area->fa_off);

    const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(dozee_sheet_smallsection));
    init_uart(uart_dev);

    struct uart_data rx_buf = {0};
    int              i      = 0;

    while (k_msgq_get(&uart_msgq, &rx_buf, K_FOREVER) == 0)
    {
        frame_frm_t frame;
        ff_err_t    ff_err = ff_process_frame(&frame, rx_buf.data, rx_buf.len);
        if (ff_err != FF_ERR_OK)
        {
            printk("Error processing frame: %d\n", ff_err);
            continue;
        }

        if (frame.soh != SOF_CHAR)
        {
            printk("Invalid start of header: %02x\n", frame.soh);
        }

        if (frame.eot != EOF_CHAR)
        {
            printk("Invalid end of transmission: %02x\n", frame.eot);
        }

        printk("Data length: %d\n", frame.len);

        if (frame.buff && frame.len == 512)
        {
            flash_img_buffered_write(&ctx, frame.buff, frame.len, false);
            printk(" (%i) Wrote %d bytes\n", i, frame.len);
        }
        else
        {
            flash_img_buffered_write(&ctx, frame.buff, frame.len, true);
            printk(" (%i) Wrote %d bytes final:\n", i, frame.len);
            printk(" Done writing\n");

            k_sleep(K_MSEC(1000));
            size_t total_size = flash_img_bytes_written(&ctx);
            printk("Total size: %d\n", total_size);

            // Check if the image is valid
            free(frame.buff);
            break;
        }

        free(frame.buff);
        memset(&rx_buf, 0, sizeof(rx_buf));
        // toggle the LED
        gpio_pin_toggle(led.port, led.pin);
        i++;
    }

    k_sleep(K_MSEC(1000));
    // Request upgrade after successful flash and check
    err = boot_request_upgrade_multi(0, BOOT_UPGRADE_PERMANENT);
    if (err)
    {
        printk("Failed to request upgrade: %d\n", err);
    }

    // Reboot the system
    sys_reboot(SYS_REBOOT_COLD);

    return 0;
}
