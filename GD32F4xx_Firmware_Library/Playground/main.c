/*!
    \file    main.c
    \brief   led spark with systick

    \version 2016-08-15, V1.0.0, firmware for GD32F4xx
    \version 2018-12-12, V2.0.0, firmware for GD32F4xx
    \version 2020-09-30, V2.1.0, firmware for GD32F4xx
    \version 2022-03-09, V3.0.0, firmware for GD32F4xx
*/

/*
    Copyright (c) 2022, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <gd32f4xx.h>
#include <gd32f470z_tk.h>
#include "systick.h"
#include "i2c.h"
#include "i2c_devices.h"
#include "enet_cfg.h"
#include "main.h"

void i2c_nvic_config(void);

int _write(int fd, char *buf, int size)
{
    (void)fd;

    for (int i = 0; i < size; i++)
    {
        while (usart_flag_get(EVAL_COM0, USART_FLAG_TBE) == RESET);
        usart_data_transmit(EVAL_COM0, buf[i]);
    }

    while (usart_flag_get(EVAL_COM0, USART_FLAG_TC) == RESET);

    return size;
}

/*!
    \brief      configure the NVIC peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_nvic_config(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(I2C0_EV_IRQn, 0, 2);
    nvic_irq_enable(I2C0_ER_IRQn, 0, 1);
}

/*!
    \brief    main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
	int i = 0;

    gd_eval_led_init(LED2);
    gd_eval_7seg_init();
    gd_eval_com_init(EVAL_COM0);
    systick_config();

    /* configure the NVIC */
    i2c_nvic_config();

    /* configure GPIO */
    i2c_gpio_config();

    /* configure I2C */
    i2c_config();
    i2c_devices_init();

    printf("Playground project\r\n");
    gd_eval_7seg_display_digits(i++);

    rtc_startup();

    i2c_probe_test();
    gd_eval_7seg_display_digits(i++);

    enet_system_setup();
    gd_eval_7seg_display_digits(i++);

    while(1) {
    	if (enet_rx_frame() > 0)
    	{
    		gd_eval_7seg_display_digits(i++);
    		if (i > 99) i = 0;
    	}
    	//enet_tx_test_frame();
    }
}
