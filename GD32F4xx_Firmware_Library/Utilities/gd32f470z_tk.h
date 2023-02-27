/*!
    \file    gd32f450z_eval.h
    \brief   definitions for GD32F470 training kit leds, keys and COM ports hardware resources

    \version 2016-08-15, V1.0.0, firmware for GD32F4xx
    \version 2018-12-12, V2.0.0, firmware for GD32F4xx
    \version 2020-09-30, V2.1.0, firmware for GD32F4xx
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

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

#ifndef GD32F470Z_TRAINING_KIT_H
#define GD32F470Z_TRAINING_KIT_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "gd32f4xx.h"

/* eval board low layer led */
#define LEDn                             2U

#define LED1_PIN                         GPIO_PIN_2
#define LED1_GPIO_PORT                   GPIOF
#define LED1_GPIO_CLK                    RCU_GPIOF

#define LED2_PIN                         GPIO_PIN_2
#define LED2_GPIO_PORT                   GPIOG
#define LED2_GPIO_CLK                    RCU_GPIOG

#define COMn                             1U
#define EVAL_COM0                        USART0
#define EVAL_COM0_CLK                    RCU_USART0

#define EVAL_COM0_TX_PIN                 GPIO_PIN_9
#define EVAL_COM0_RX_PIN                 GPIO_PIN_10

#define EVAL_COM0_GPIO_PORT              GPIOA
#define EVAL_COM0_GPIO_CLK               RCU_GPIOA
#define EVAL_COM0_AF                     GPIO_AF_7

#define SEG7_IND_NUM                     2U
#define SEG7_LEDS                        8U
#define SEG7_MINUS_ITM                   27U

#define KEYn                             2U

/* wakeup push-button */
#define WAKEUP_KEY_PIN                   GPIO_PIN_0
#define WAKEUP_KEY_GPIO_PORT             GPIOA
#define WAKEUP_KEY_GPIO_CLK              RCU_GPIOA
#define WAKEUP_KEY_EXTI_LINE             EXTI_0
#define WAKEUP_KEY_EXTI_PORT_SOURCE      EXTI_SOURCE_GPIOA
#define WAKEUP_KEY_EXTI_PIN_SOURCE       EXTI_SOURCE_PIN0
#define WAKEUP_KEY_EXTI_IRQn             EXTI0_IRQn

/* user push-button */
#define USER_KEY_PIN                     GPIO_PIN_0
#define USER_KEY_GPIO_PORT               GPIOC
#define USER_KEY_GPIO_CLK                RCU_GPIOC
#define USER_KEY_EXTI_LINE               EXTI_0
#define USER_KEY_EXTI_PORT_SOURCE        EXTI_SOURCE_GPIOC
#define USER_KEY_EXTI_PIN_SOURCE         EXTI_SOURCE_PIN0
#define USER_KEY_EXTI_IRQn               EXTI0_IRQn

/* exported types */
typedef enum
{
    LED1 = 0,
    LED2 = 1
} led_typedef_enum;

typedef enum
{
    KEY_WAKEUP = 0,
    KEY_USER = 1
} key_typedef_enum;

typedef enum
{
    KEY_MODE_GPIO = 0,
    KEY_MODE_EXTI = 1
} keymode_typedef_enum;

typedef enum
{
    SEG7_LED_DEV1 = 0,
    SEG7_LED_DEV2 = 1
} seg7_typedef_enum;

typedef struct
{
    uint32_t gpio_periph;
    uint32_t pin;
} seg7_led_gpio;

/* function declarations */
/* configures led GPIO */
void gd_eval_led_init(led_typedef_enum lednum);
/* turn on selected led */
void gd_eval_led_on(led_typedef_enum lednum);
/* turn off selected led */
void gd_eval_led_off(led_typedef_enum lednum);
/* toggle the selected led */
void gd_eval_led_toggle(led_typedef_enum lednum);
/* configure key */
void gd_eval_key_init(key_typedef_enum key_num, keymode_typedef_enum key_mode);
/* return the selected button state */
uint8_t gd_eval_key_state_get(key_typedef_enum button);
/* configure COM port */
void gd_eval_com_init(uint32_t com);
/* configures 7segment LED GPIO */
void gd_eval_7seg_init(void);
/* set 7segment value */
void gd_eval_7seg_set_value(seg7_typedef_enum devnum, uint8_t value);
/* set 7segment digit */
void gd_eval_7seg_display_digits(int8_t value);


#ifdef __cplusplus
}
#endif

#endif /* GD32F470Z_TRAINING_KIT_H */
