/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/debug/coredump.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(Lesson2_Exercise2, LOG_LEVEL_INF);

void crash_function(uint32_t *addr)
{
    LOG_INF("Button pressed at %" PRIu32, k_cycle_get_32());
    LOG_INF("Coredump: %s", CONFIG_BOARD);

#if !defined(CONFIG_CPU_CORTEX_M)
    /* For null pointer reference */
    *addr = 0;
#else
    ARG_UNUSED(addr);
    /* Dereferencing null-pointer in TrustZone-enabled
     * builds may crash the system, so use, instead an
     * undefined instruction to trigger a CPU fault.
     */
    __asm__ volatile("udf #0" : : :);
#endif
}

int main(void)
{
    k_sleep(K_MSEC(1000));
    crash_function(0);
    return 0;
}
