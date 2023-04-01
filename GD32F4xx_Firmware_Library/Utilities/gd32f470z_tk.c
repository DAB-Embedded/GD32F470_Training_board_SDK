/*!
    \file    gd32f450z_eval.c
    \brief   firmware functions to manage leds, keys, COM ports

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

#include <stdio.h>
#include <gd32f470z_tk.h>

/* private variables */
static uint32_t GPIO_PORT[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT};
static uint32_t GPIO_PIN[LEDn] = {LED1_PIN, LED2_PIN};

static rcu_periph_enum COM_CLK[COMn] = {EVAL_COM0_CLK};
static uint32_t COM_TX_PIN[COMn] = {EVAL_COM0_TX_PIN};
static uint32_t COM_RX_PIN[COMn] = {EVAL_COM0_RX_PIN};

static rcu_periph_enum GPIO_CLK[LEDn] = {LED1_GPIO_CLK, LED2_GPIO_CLK};

static uint32_t KEY_PORT[KEYn] = {WAKEUP_KEY_GPIO_PORT,
                                  USER_KEY_GPIO_PORT};
static uint32_t KEY_PIN[KEYn] = {WAKEUP_KEY_PIN, USER_KEY_PIN};
static rcu_periph_enum KEY_CLK[KEYn] = {WAKEUP_KEY_GPIO_CLK,
                                        USER_KEY_GPIO_CLK};
static exti_line_enum KEY_EXTI_LINE[KEYn] = {WAKEUP_KEY_EXTI_LINE,
                                             USER_KEY_EXTI_LINE};
static uint8_t KEY_PORT_SOURCE[KEYn] = {WAKEUP_KEY_EXTI_PORT_SOURCE,
                                        USER_KEY_EXTI_PORT_SOURCE};
static uint8_t KEY_PIN_SOURCE[KEYn] = {WAKEUP_KEY_EXTI_PIN_SOURCE,
                                       USER_KEY_EXTI_PIN_SOURCE};
static uint8_t KEY_IRQn[KEYn] = {WAKEUP_KEY_EXTI_IRQn,
                                 USER_KEY_EXTI_IRQn};
const  seg7_led_gpio SEG7_LED_DEVICE1[SEG7_LEDS] = {
                                 {GPIOF, GPIO_PIN_14},
                                 {GPIOF, GPIO_PIN_15},
                                 {GPIOF, GPIO_PIN_3 },
                                 {GPIOF, GPIO_PIN_4 },
                                 {GPIOF, GPIO_PIN_5 },
                                 {GPIOF, GPIO_PIN_13},
                                 {GPIOF, GPIO_PIN_12},
                                 {GPIOF, GPIO_PIN_2 }
                                };

const  seg7_led_gpio SEG7_LED_DEVICE2[SEG7_LEDS] = {
                                 {GPIOE, GPIO_PIN_7 },
                                 {GPIOE, GPIO_PIN_8 },
                                 {GPIOG, GPIO_PIN_3 },
                                 {GPIOG, GPIO_PIN_4 },
                                 {GPIOG, GPIO_PIN_5 },
                                 {GPIOG, GPIO_PIN_1 },
                                 {GPIOG, GPIO_PIN_0 },
                                 {GPIOG, GPIO_PIN_2 }
                                };
/*!
    \brief    Init 2x 7 segment LED displays
    \retval     none
*/
void gd_eval_7seg_init(void)
{
    int i;

    /* enable the led clock */
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_GPIOF);
    rcu_periph_clock_enable(RCU_GPIOG);
    /* configure led GPIO port */
    for (i = 0; i < SEG7_LEDS; i++)
    {
        /* configure led1 GPIO port */
        gpio_mode_set(SEG7_LED_DEVICE1[i].gpio_periph, GPIO_MODE_OUTPUT, \
            GPIO_PUPD_NONE, SEG7_LED_DEVICE1[i].pin);
        gpio_output_options_set(SEG7_LED_DEVICE1[i].gpio_periph, \
            GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,SEG7_LED_DEVICE1[i].pin);

        GPIO_BOP(SEG7_LED_DEVICE1[i].gpio_periph) = SEG7_LED_DEVICE1[i].pin;

        /* configure led2 GPIO port */
        gpio_mode_set(SEG7_LED_DEVICE2[i].gpio_periph, GPIO_MODE_OUTPUT, \
            GPIO_PUPD_NONE, SEG7_LED_DEVICE2[i].pin);
        gpio_output_options_set(SEG7_LED_DEVICE2[i].gpio_periph, \
            GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,SEG7_LED_DEVICE2[i].pin);

        GPIO_BOP(SEG7_LED_DEVICE2[i].gpio_periph) = SEG7_LED_DEVICE2[i].pin;
    }
}

/*!
    \brief    draw character on display
    \param[in]  devnum: specify the 7Seg Led device to be used
      \arg        SEG7_LED_DEV1
      \arg        SEG7_LED_DEV2
    \param[in]  value [0..27] - value to display (0..9, letters)
    \retval     none
*/
void gd_eval_7seg_set_value(seg7_typedef_enum devnum, uint8_t value)
{
#if 1
    const uint32_t seg7_1_patterns_gpioe[]   = \
       {0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000};
    const uint32_t seg7_1_patterns_gpiof[]   = \
       {0xe0381004, 0x80087034, 0xd030200c, 0xd0182024,
        0xb0084034, 0x70188024, 0x70388004, 0xc0083034,
        0xf0380004, 0xf0180024, 0xf0280014, 0x3038c004,
        0x6030900c, 0x90386004, 0x7030800c, 0x7020801c,
        0x60389004, 0xb0284014, 0x2020d01c, 0x2030d00c,
        0x1028e014, 0x1038e004, 0xf020001c, 0x1020e01c,
        0x3030c00c, 0xa0385004, 0x0038f004, 0x1000e03c};
    const uint32_t seg7_1_patterns_gpiog[]   = \
       {0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000};

    const uint32_t seg7_2_patterns_gpioe[]   = \
       {0x01800000, 0x01000080, 0x01800000, 0x01800000,
        0x01000080, 0x00800100, 0x00800100, 0x01800000,
        0x01800000, 0x01800000, 0x01800000, 0x00000180,
        0x00800100, 0x01000080, 0x00800100, 0x00800100,
        0x00800100, 0x01000080, 0x00000180, 0x00000180,
        0x00000180, 0x00000180, 0x01800000, 0x00000180,
        0x00000180, 0x01000080, 0x00000180, 0x00000180};
    const uint32_t seg7_2_patterns_gpiof[]   = \
       {0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000};
    const uint32_t seg7_2_patterns_gpiog[]   = \
       {0x003a0005, 0x00080037, 0x0031000e, 0x00190026,
        0x000b0034, 0x001b0024, 0x003b0004, 0x00080037,
        0x003b0004, 0x001b0024, 0x002b0014, 0x003b0004,
        0x0032000d, 0x00390006, 0x0033000c, 0x0023001c,
        0x003a0005, 0x002b0014, 0x0022001d, 0x0032000d,
        0x00290016, 0x00390006, 0x0023001c, 0x0021001e,
        0x0033000c, 0x003a0005, 0x00380007, 0x0001003e};
//    0     1     2     3     4     5     6     7     8     9
//    A     b     C     d     E     F     G     H     I     L
//    n     o     P     r     t     U     u     -

    if (value > 27)
    {
        return;
    }

    if(SEG7_LED_DEV1 == devnum)
    {
        GPIO_BOP(GPIOE) = seg7_1_patterns_gpioe[value];
        GPIO_BOP(GPIOF) = seg7_1_patterns_gpiof[value];
        GPIO_BOP(GPIOG) = seg7_1_patterns_gpiog[value];
    } else {
        GPIO_BOP(GPIOE) = seg7_2_patterns_gpioe[value];
        GPIO_BOP(GPIOF) = seg7_2_patterns_gpiof[value];
        GPIO_BOP(GPIOG) = seg7_2_patterns_gpiog[value];
    }
#endif

#if 0
    int i;
    uint32_t gpioe_out = 0, gpiof_out = 0, gpiog_out = 0;
    uint32_t gpioe_mask = 0, gpiof_mask = 0, gpiog_mask = 0;
    const uint8_t seg7_pattern[] = \
        {0x3F, 0x06, 0x5B, 0x4F, 0x66,
         0x6D, 0x7D, 0x07, 0x7F, 0x6F,
         0x77, 0x7C, 0x39, 0x5E, 0x79,
         0x71, 0x3D, 0x76, 0x30, 0x38,
         0x54, 0x5C, 0x73, 0x50, 0x78,
         0x3E, 0x1C, 0x40};
 //    0     1     2     3     4     5     6     7     8     9
 //    A     b     C     d     E     F     G     H     I     L
 //    n     o     P     r     t     U     u     -

    if (value > 27)
    {
        return;
    }

    if(SEG7_LED_DEV1 == devnum)
    {
        for (i = 0; i < SEG7_LEDS; i++)
        {
            GPIO_BOP(SEG7_LED_DEVICE1[i].gpio_periph) = SEG7_LED_DEVICE1[i].pin;
        }

        for (i = 0; i < SEG7_LEDS; i++)
        {
            if ((seg7_pattern[value] & (1 << i)) != 0)
            {
                GPIO_BC(SEG7_LED_DEVICE1[i].gpio_periph) = SEG7_LED_DEVICE1[i].pin;
                if (SEG7_LED_DEVICE1[i].gpio_periph == GPIOE)
                    gpioe_out |= SEG7_LED_DEVICE1[i].pin;
                if (SEG7_LED_DEVICE1[i].gpio_periph == GPIOF)
                    gpiof_out |= SEG7_LED_DEVICE1[i].pin;
                if (SEG7_LED_DEVICE1[i].gpio_periph == GPIOG)
                    gpiog_out |= SEG7_LED_DEVICE1[i].pin;
            }

            if (SEG7_LED_DEVICE1[i].gpio_periph == GPIOE)
                gpioe_mask |= SEG7_LED_DEVICE1[i].pin;
            if (SEG7_LED_DEVICE1[i].gpio_periph == GPIOF)
                gpiof_mask |= SEG7_LED_DEVICE1[i].pin;
            if (SEG7_LED_DEVICE1[i].gpio_periph == GPIOG)
                gpiog_mask |= SEG7_LED_DEVICE1[i].pin;
        }
    } else {
        for (i = 0; i < SEG7_LEDS; i++)
        {
            GPIO_BOP(SEG7_LED_DEVICE2[i].gpio_periph) = SEG7_LED_DEVICE2[i].pin;
        }

        for (i = 0; i < SEG7_LEDS; i++)
        {
            if ((seg7_pattern[value] & (1 << i)) != 0)
            {
                GPIO_BC(SEG7_LED_DEVICE2[i].gpio_periph) = SEG7_LED_DEVICE2[i].pin;
                if (SEG7_LED_DEVICE2[i].gpio_periph == GPIOE)
                    gpioe_out |= SEG7_LED_DEVICE2[i].pin;
                if (SEG7_LED_DEVICE2[i].gpio_periph == GPIOF)
                    gpiof_out |= SEG7_LED_DEVICE2[i].pin;
                if (SEG7_LED_DEVICE2[i].gpio_periph == GPIOG)
                    gpiog_out |= SEG7_LED_DEVICE2[i].pin;
            }

            if (SEG7_LED_DEVICE2[i].gpio_periph == GPIOE)
                gpioe_mask |= SEG7_LED_DEVICE2[i].pin;
            if (SEG7_LED_DEVICE2[i].gpio_periph == GPIOF)
                gpiof_mask |= SEG7_LED_DEVICE2[i].pin;
            if (SEG7_LED_DEVICE2[i].gpio_periph == GPIOG)
                gpiog_mask |= SEG7_LED_DEVICE2[i].pin;
        }
    }

    printf("0x%08x,\r\n", (gpiog_out << 16) | ((~gpiog_out & 0xFFFFU) & gpiog_mask));
#endif
}

/*!
    \brief    draw value on display
    \param[in]  value [-9..100] - value to display, 100 - means HI
    \retval     none
*/
void gd_eval_7seg_display_digits(int8_t value)
{
    if ((value >= 0) && (value < 100))
    {
        gd_eval_7seg_set_value(SEG7_LED_DEV1, value / 10);
        gd_eval_7seg_set_value(SEG7_LED_DEV2, value % 10);
    } else if ((value < 0) && (value > -10))
    {
        gd_eval_7seg_set_value(SEG7_LED_DEV1, SEG7_MINUS_ITM);
        gd_eval_7seg_set_value(SEG7_LED_DEV2, abs(value % 10));
    } else if (value == 100)
    {
        gd_eval_7seg_set_value(SEG7_LED_DEV1, 17);
        gd_eval_7seg_set_value(SEG7_LED_DEV2, 18);
    }
}

/*!
    \brief    configure led GPIO
    \param[in]  lednum: specify the Led to be configured
      \arg        LED1
      \arg        LED2
    \param[out] none
    \retval     none
*/
void  gd_eval_led_init (led_typedef_enum lednum)
{
    /* enable the led clock */
    rcu_periph_clock_enable(GPIO_CLK[lednum]);
    /* configure led GPIO port */
    gpio_mode_set(GPIO_PORT[lednum], GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,GPIO_PIN[lednum]);
    gpio_output_options_set(GPIO_PORT[lednum], GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN[lednum]);

    GPIO_BC(GPIO_PORT[lednum]) = GPIO_PIN[lednum];
}

/*!
    \brief    turn on selected led
    \param[in]  lednum: specify the Led to be turned on
      \arg        LED1
      \arg        LED2
    \param[out] none
    \retval     none
*/
void gd_eval_led_on(led_typedef_enum lednum)
{
    GPIO_BOP(GPIO_PORT[lednum]) = GPIO_PIN[lednum];
}

/*!
    \brief    turn off selected led
    \param[in]  lednum: specify the Led to be turned off
      \arg        LED1
      \arg        LED2
    \param[out] none
    \retval     none
*/
void gd_eval_led_off(led_typedef_enum lednum)
{
    GPIO_BC(GPIO_PORT[lednum]) = GPIO_PIN[lednum];
}

/*!
    \brief    toggle selected led
    \param[in]  lednum: specify the Led to be toggled
      \arg        LED1
      \arg        LED2
    \param[out] none
    \retval     none
*/
void gd_eval_led_toggle(led_typedef_enum lednum)
{
    GPIO_TG(GPIO_PORT[lednum]) = GPIO_PIN[lednum];
}

/*!
    \brief    configure key
    \param[in]  key_num: specify the key to be configured
      \arg        KEY_WAKEUP: wakeup key
      \arg        KEY_USER: user key
    \param[in]  key_mode: specify button mode
      \arg        KEY_MODE_GPIO: key will be used as simple IO
      \arg        KEY_MODE_EXTI: key will be connected to EXTI line with interrupt
    \param[out] none
    \retval     none
*/
void gd_eval_key_init(key_typedef_enum key_num, keymode_typedef_enum key_mode)
{
    /* enable the key clock */
    rcu_periph_clock_enable(KEY_CLK[key_num]);
    rcu_periph_clock_enable(RCU_SYSCFG);

    /* configure button pin as input */
    gpio_mode_set(KEY_PORT[key_num], GPIO_MODE_INPUT, GPIO_PUPD_NONE,KEY_PIN[key_num]);

    if (key_mode == KEY_MODE_EXTI) {
        /* enable and set key EXTI interrupt to the lowest priority */
        nvic_irq_enable(KEY_IRQn[key_num], 2U, 0U);

        /* connect key EXTI line to key GPIO pin */
        syscfg_exti_line_config(KEY_PORT_SOURCE[key_num], KEY_PIN_SOURCE[key_num]);

        /* configure key EXTI line */
        exti_init(KEY_EXTI_LINE[key_num], EXTI_INTERRUPT, EXTI_TRIG_FALLING);
        exti_interrupt_flag_clear(KEY_EXTI_LINE[key_num]);
    }
}

/*!
    \brief    return the selected button state
    \param[in]  button: specify the button to be checked
      \arg        KEY_WAKEUP: wakeup key
      \arg        KEY_USER: user key
    \param[out] none
    \retval     the button GPIO pin value
*/
uint8_t gd_eval_key_state_get(key_typedef_enum button)
{
    return gpio_input_bit_get(KEY_PORT[button], KEY_PIN[button]);
}

/*!
    \brief    configure COM port
    \param[in]  COM: COM on the board
      \arg        EVAL_COM0: COM on the board
    \param[out] none
    \retval     none
*/
void gd_eval_com_init(uint32_t com)
{
    /* enable GPIO clock */
    uint32_t COM_ID = 0;
    if(EVAL_COM0 == com)
    {
        COM_ID = 0U;
    }

    rcu_periph_clock_enable( EVAL_COM0_GPIO_CLK);

    /* enable USART clock */
    rcu_periph_clock_enable(COM_CLK[COM_ID]);

    /* connect port to USARTx_Tx */
    gpio_af_set(EVAL_COM0_GPIO_PORT, EVAL_COM0_AF, COM_TX_PIN[COM_ID]);

    /* connect port to USARTx_Rx */
    gpio_af_set(EVAL_COM0_GPIO_PORT, EVAL_COM0_AF, COM_RX_PIN[COM_ID]);

    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(EVAL_COM0_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP,COM_TX_PIN[COM_ID]);
    gpio_output_options_set(EVAL_COM0_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,COM_TX_PIN[COM_ID]);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(EVAL_COM0_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP,COM_RX_PIN[COM_ID]);
    gpio_output_options_set(EVAL_COM0_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,COM_RX_PIN[COM_ID]);

    /* USART configure */
    usart_deinit(com);
    usart_baudrate_set(com,115200U);
    usart_parity_config(com,USART_PM_NONE);
    usart_word_length_set(com,USART_WL_8BIT);
    usart_receive_config(com, USART_RECEIVE_ENABLE);
    usart_transmit_config(com, USART_TRANSMIT_ENABLE);
    usart_enable(com);
}
