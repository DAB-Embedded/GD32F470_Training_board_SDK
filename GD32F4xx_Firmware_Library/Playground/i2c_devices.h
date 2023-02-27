/*!
    \file    i2c_devices.h
    \brief   the header file of I2c devices

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

#ifndef I2C_DEVICES__H
#define I2C_DEVICES__H

#include "gd32f4xx.h"

typedef enum {
    I2C_SEND_ADDRESS_FIRST = 0,
    I2C_CLEAR_ADDRESS_FLAG_FIRST,
    I2C_TRANSMIT_DATA,
    I2C_STOP
} i2c_process_enum;

#define I2C_OK         0
#define I2C_FAIL       1
#define I2CX           I2C0

/* function declarations */
/* I2C read and write functions */
uint8_t i2c_probe_test(void);
/* initialize peripherals used by the I2C device driver */
void i2c_devices_init(void);
/* write buffer of data to the I2C device by interrupt */
void i2c_buffer_write_interrupt(uint8_t sa, uint8_t* p_buffer, uint16_t number_of_byte);
/* read data from the device by interrupt */
void i2c_buffer_read_interrupt(uint8_t sa, uint8_t *p_buffer, uint16_t number_of_byte);
/* wait for device standby state */
void i2c_wait_standby_state(void);

#endif /* I2C_DEVICES__H */