/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-10-25     zylx         first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <dfs_fs.h>

/* defined the LED0 pin: PE3 */
#define LED0_PIN GET_PIN(E, 3)

/* TODO */
const struct dfs_mount_tbl mount_table[2] =
    {
        {"sd0", "/", "lfs", 0, 0},
        {0}
};

int main(void)
{
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

    rt_sdio_init();

    while (1)
    {
        //extern void cdc_acm_data_send_with_dtr_test(uint8_t busid);
        //cdc_acm_data_send_with_dtr_test(0);
        //extern void audio_v1_test(uint8_t busid);
        //audio_v1_test(0);
        //rt_thread_mdelay(10);
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
    return RT_EOK;
}
