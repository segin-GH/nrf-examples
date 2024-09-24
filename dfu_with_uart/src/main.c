#include "zephyr/sys/printk.h"
#include <stdbool.h>
#include <zephyr/dfu/flash_img.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>

#define LED_NODE      DT_ALIAS(led1)
#define FW_CHUNK_SIZE 512

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

#define MCUBOOT_PARTITION_ID FLASH_AREA_ID(mcuboot)
#define IMAGE_0_PARTITION_ID FLASH_AREA_ID(image_0)
#define IMAGE_1_PARTITION_ID FLASH_AREA_ID(image_1)
#define STORAGE_PARTITION_ID FLASH_AREA_ID(storage)

void print_partition_info(int partition_id, const char *label)
{
    const struct flash_area *fa;
    int                      err = flash_area_open(partition_id, &fa);
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

void get_the_info_of_current_img(void)
{
    struct mcuboot_img_header hdr;

    int err = boot_read_bank_header(IMAGE_0_PARTITION_ID, &hdr, sizeof(hdr));
    if (err)
    {
        printk("Unable to get header data\n");
        return;
    }

    printk("Mcuboot version %i\n", hdr.mcuboot_version);
    printk("FW version v%i.%i.%i\n", hdr.h.v1.sem_ver.major, hdr.h.v1.sem_ver.minor, hdr.h.v1.sem_ver.revision);
}

int main(void)
{
    printk("Hello World!\n");
    int err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

    struct flash_img_context ctx = {0};

    err = flash_img_init_id(&ctx, MCUBOOT_PARTITION_ID);
    if (err)
    {
        printk("Flash img failed\n");
        return -1;
    }

    printk("Flash img initialized\n");
    printk("Flash size: %d\n", ctx.flash_area->fa_size);
    printk("Flash offset: %ld\n", ctx.flash_area->fa_off);

    get_the_info_of_current_img();

    print_partition_info(MCUBOOT_PARTITION_ID, "mcuboot");
    print_partition_info(IMAGE_0_PARTITION_ID, "image-0");
    print_partition_info(IMAGE_1_PARTITION_ID, "image-1");
    print_partition_info(STORAGE_PARTITION_ID, "storage");

    while (true)
    {
        err = gpio_pin_toggle_dt(&led);
        k_msleep(1000);
    }
    return 0;
}
