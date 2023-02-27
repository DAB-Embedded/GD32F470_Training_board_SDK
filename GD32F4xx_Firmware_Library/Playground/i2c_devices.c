/*!
    \file    at24cxx.c
    \brief   the read and write function file

    \version 2022-01-15, V1.0.0, firmware for GD32F4xx
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

#include "i2c_devices.h"
#include "i2c.h"
#include <stdio.h>
#include <gd32f470z_tk.h>

#define BUFFER_SIZE 256

volatile uint16_t i2c_address;
uint8_t i2c_buffer_write[BUFFER_SIZE];
uint8_t i2c_buffer_read[BUFFER_SIZE];
volatile uint8_t  *i2c_write;
volatile uint8_t  *i2c_read;
volatile uint16_t i2c_nbytes;
volatile uint8_t  i2c_process_flag = 0;
volatile uint8_t  i2c_nack_flag = 0;

#define LTR303_SA       0x29
#define LIS2DH12_SA     0x18

uint32_t ltr303_read_value(void)
{
    i2c_buffer_write[0] = 0x88;
    i2c_buffer_write_interrupt(LTR303_SA, i2c_buffer_write, 1);

    i2c_buffer_read_interrupt(LTR303_SA, i2c_buffer_read, 4);

    return i2c_buffer_read[0] | i2c_buffer_read[1] << 8 | i2c_buffer_read[2] << 16 | i2c_buffer_read[3] << 24;
}

/*!
    \brief      I2C read and write functions
    \param[in]  none
    \param[out] none
    \retval     I2C_OK or I2C_FAIL
*/
uint8_t i2c_probe_test(void)
{
    int i;

    printf("\r\nSearching for I2C devices...\r\n");

    for (i = 0x01; i < 0x7F; i++)
    {
        i2c_buffer_read_interrupt(i, i2c_buffer_read, 1);
        if (i2c_nack_flag == 0)
           printf("Found slave with ID %x\r\n", i);
    }

    // Light sensor init
    i2c_buffer_write[0] = 0x80;
    i2c_buffer_write[1] = 0x02;

    i2c_buffer_write_interrupt(LTR303_SA, i2c_buffer_write, 2);
    delay_1ms(50);

    i2c_buffer_write[0] = 0x80;
    i2c_buffer_write[1] = 0x01 | (3 << 2);
    i2c_buffer_write_interrupt(LTR303_SA, i2c_buffer_write, 2);

    i2c_buffer_write[0] = 0x85;
    i2c_buffer_write[1] = 0x03 | (3 << 3);
    i2c_buffer_write_interrupt(LTR303_SA, i2c_buffer_write, 2);

#if 0
    delay_1ms(650);
    uint32_t light_val32 = ltr303_read_value();
    volatile uint32_t vals32;

    while(1) {
        light_val32 = ltr303_read_value();
        vals32 = (light_val32&0xFFFF)/50;
        gd_eval_7seg_display_digits(vals32);
    }
#endif
    return I2C_OK;
}

/*!
    \brief      initialize peripherals used by the I2C i2c driver
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_devices_init(void)
{
    i2c_address = 0;
    i2c_bus_reset();
}

/*!
    \brief      write buffer of data to the I2C i2c by interrupt
    \param[in]  sa: device slave address
    \param[in]  p_buffer: pointer to the buffer  containing the data to be written to the i2c
    \param[in]  number_of_byte: number of bytes to write to the i2c
    \param[out] none
    \retval     none
*/
void i2c_buffer_write_interrupt(uint8_t sa, uint8_t *p_buffer, uint16_t number_of_byte)
{
    i2c_address = sa << 1;
    i2c_nbytes  = number_of_byte;
    i2c_write   = p_buffer;

    /* write data by interrupt */
    i2c_process_flag = 0;
    i2c_nack_flag    = 0;

    /* enable the I2CX interrupt */
    i2c_interrupt_enable(I2CX, I2C_INT_ERR);
    i2c_interrupt_enable(I2CX, I2C_INT_EV);
    i2c_interrupt_enable(I2CX, I2C_INT_BUF);

    /* the master waits until the I2C bus is idle */
    while(i2c_flag_get(I2CX, I2C_FLAG_I2CBSY));

    /* enable acknowledge */
    i2c_ack_config(I2CX, I2C_ACK_ENABLE);

    /* the master sends a start condition to I2C bus */
    i2c_start_on_bus(I2CX);
    while((i2c_nbytes > 0)) {
    }

    /* wait until I2C bus is idle */
    while(i2c_flag_get(I2CX, I2C_FLAG_I2CBSY));
}

/*!
    \brief      read data from the i2c by interrupt
    \param[in]  p_buffer: pointer to the buffer that receives the data read from the i2c
    \param[in]  read_address: i2c's internal address to start reading from
    \param[in]  number_of_byte: number of bytes to reads from the i2c
    \param[out] none
    \retval     none
*/
void i2c_buffer_read_interrupt(uint8_t sa, uint8_t *p_buffer, uint16_t number_of_byte)
{
    i2c_read = p_buffer;
    i2c_nbytes = number_of_byte;
    i2c_address = sa << 1;

    i2c_process_flag = SET;
    i2c_nack_flag = 0;

    /* enable acknowledge */
    i2c_ack_config(I2CX, I2C_ACK_ENABLE);
    /* enable the I2CX interrupt */
    i2c_interrupt_enable(I2CX, I2C_INT_ERR);
    i2c_interrupt_enable(I2CX, I2C_INT_EV);
    i2c_interrupt_enable(I2CX, I2C_INT_BUF);
    /* wait until I2C bus is idle */
    while(i2c_flag_get(I2CX, I2C_FLAG_I2CBSY));
    if(2 == number_of_byte) {
        i2c_ackpos_config(I2CX, I2C_ACKPOS_NEXT);
    }
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2CX);
    while((i2c_nbytes > 0)) {
    }

    /* wait until I2C bus is idle */
    while(i2c_flag_get(I2CX, I2C_FLAG_I2CBSY));
}

/*!
    \brief      wait for i2c standby state
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_wait_standby_state(void)
{
    __IO uint32_t val = 0;

    while(1) {
        /* wait until I2C bus is idle */
        while(i2c_flag_get(I2CX, I2C_FLAG_I2CBSY));

        /* send a start condition to I2C bus */
        i2c_start_on_bus(I2CX);

        /* wait until SBSEND bit is set */
        while(!i2c_flag_get(I2CX, I2C_FLAG_SBSEND));

        /* send slave address to I2C bus */
        i2c_master_addressing(I2CX, i2c_address, I2C_TRANSMITTER);

        /* keep looping till the Address is acknowledged or the AE flag is set (address not acknowledged at time) */
        do {
            /* get the current value of the I2C_STAT0 register */
            val = I2C_STAT0(I2CX);
        } while(0 == (val & (I2C_STAT0_ADDSEND | I2C_STAT0_AERR)));

        /* check if the ADDSEND flag has been set */
        if(val & I2C_STAT0_ADDSEND) {
            /* clear ADDSEND flag */
            i2c_flag_clear(I2CX, I2C_FLAG_ADDSEND);
            /* send a stop condition to I2C bus */
            i2c_stop_on_bus(I2CX);
            /* exit the function */
            return ;
        } else {
            /* clear the bit of AERR */
            i2c_flag_clear(I2CX, I2C_FLAG_AERR);
        }
        /* send a stop condition to I2C bus */
        i2c_stop_on_bus(I2CX);
        /* wait until the stop condition is finished */
        while(I2C_CTL0(I2CX) & I2C_CTL0_STOP);
    }
}
