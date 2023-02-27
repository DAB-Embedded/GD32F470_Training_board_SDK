/**
  ******************************************************************************
  * @file      startup_gd32f450_470.s
  * @author    DAB-Embedded
  * @brief     GD32F470xx Devices vector table for GCC based toolchains.
  ******************************************************************************
  * @version   2022-03-09, V3.0.0, firmware for GD32F4xx
  */

  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb

.global  g_pfnVectors
.global  Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word  _sidata
/* start address for the .data section. defined in linker script */
.word  _sdata
/* end address for the .data section. defined in linker script */
.word  _edata
/* start address for the .bss section. defined in linker script */
.word  _sbss
/* end address for the .bss section. defined in linker script */
.word  _ebss
/* stack used for SystemInit_ExtMemCtl; always internal RAM used */

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 * @param  None
 * @retval : None
*/

  .section  .text.Reset_Handler
  .weak  Reset_Handler
  .type  Reset_Handler, %function
Reset_Handler:
  ldr   sp, =_estack     /* set stack pointer */

/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit

/* Zero fill the bss segment. */
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  str  r3, [r2]
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss

/* Call the clock system initialization function.*/
  bl  SystemInit
/* Call static constructors */
  bl __libc_init_array
/* Call the application's entry point.*/
  bl  main
  bx  lr
.size  Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 * @param  None
 * @retval None
*/
    .section  .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b  Infinite_Loop
  .size  Default_Handler, .-Default_Handler
/******************************************************************************
*
* The minimal vector table for a Cortex M3. Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
*******************************************************************************/
   .section  .isr_vector,"a",%progbits
  .type  g_pfnVectors, %object
  .size  g_pfnVectors, .-g_pfnVectors

  g_pfnVectors:
  .word  _estack
  .word  Reset_Handler

  .word  NMI_Handler
  .word  HardFault_Handler
  .word  MemManage_Handler
  .word  BusFault_Handler
  .word  UsageFault_Handler
  .word  0
  .word  0
  .word  0
  .word  0
  .word  SVC_Handler
  .word  DebugMon_Handler
  .word  0
  .word  PendSV_Handler
  .word  SysTick_Handler

  /* External Interrupts */
  .word     WWDGT_IRQHandler                   /* Window WatchDog              */
  .word     LVD_IRQHandler                    /* PVD through EXTI Line detection */
  .word     TAMPER_STAMP_IRQHandler             /* Tamper and TimeStamps through the EXTI line */
  .word     RTC_WKUP_IRQHandler               /* RTC Wakeup through the EXTI line */
  .word     FMC_IRQHandler                  /* FLASH                        */
  .word     RCU_CTC_IRQHandler                    /* RCC                          */
  .word     EXTI0_IRQHandler                  /* EXTI Line0                   */
  .word     EXTI1_IRQHandler                  /* EXTI Line1                   */
  .word     EXTI2_IRQHandler                  /* EXTI Line2                   */
  .word     EXTI3_IRQHandler                  /* EXTI Line3                   */
  .word     EXTI4_IRQHandler                  /* EXTI Line4                   */
  .word     DMA0_Channel0_IRQHandler           /* DMA1 Stream 0                */
  .word     DMA0_Channel1_IRQHandler           /* DMA1 Stream 1                */
  .word     DMA0_Channel2_IRQHandler           /* DMA1 Stream 2                */
  .word     DMA0_Channel3_IRQHandler           /* DMA1 Stream 3                */
  .word     DMA0_Channel4_IRQHandler           /* DMA1 Stream 4                */
  .word     DMA0_Channel5_IRQHandler           /* DMA1 Stream 5                */
  .word     DMA0_Channel6_IRQHandler           /* DMA1 Stream 6                */
  .word     ADC_IRQHandler                    /* ADC1, ADC2 and ADC3s         */
  .word     CAN0_TX_IRQHandler                /* CAN1 TX                      */
  .word     CAN0_RX0_IRQHandler               /* CAN1 RX0                     */
  .word     CAN0_RX1_IRQHandler               /* CAN1 RX1                     */
  .word     CAN0_EWMC_IRQHandler               /* CAN1 SCE                     */
  .word     EXTI5_9_IRQHandler                /* External Line[9:5]s          */
  .word     TIMER0_BRK_TIMER8_IRQHandler          /* TIM1 Break and TIM9          */
  .word     TIMER0_UP_TIMER9_IRQHandler          /* TIM1 Update and TIM10        */
  .word     TIMER0_TRG_CMT_TIMER10_IRQHandler     /* TIM1 Trigger and Commutation and TIM11 */
  .word     TIMER0_Channel_IRQHandler                /* TIM1 Capture Compare         */
  .word     TIMER1_IRQHandler                   /* TIM2                         */
  .word     TIMER2_IRQHandler                   /* TIM3                         */
  .word     TIMER3_IRQHandler                   /* TIM4                         */
  .word     I2C0_EV_IRQHandler                /* I2C1 Event                   */
  .word     I2C0_ER_IRQHandler                /* I2C1 Error                   */
  .word     I2C1_EV_IRQHandler                /* I2C2 Event                   */
  .word     I2C1_ER_IRQHandler                /* I2C2 Error                   */
  .word     SPI0_IRQHandler                   /* SPI1                         */
  .word     SPI1_IRQHandler                   /* SPI2                         */
  .word     USART0_IRQHandler                 /* USART1                       */
  .word     USART1_IRQHandler                 /* USART2                       */
  .word     USART2_IRQHandler                 /* USART3                       */
  .word     EXTI10_15_IRQHandler              /* External Line[15:10]s        */
  .word     RTC_Alarm_IRQHandler              /* RTC Alarm (A and B) through EXTI Line */
  .word     USBFS_WKUP_IRQHandler            /* USB OTG FS Wakeup through EXTI line */
  .word     TIMER7_BRK_TIMER11_IRQHandler         /* TIM8 Break and TIM12         */
  .word     TIMER7_UP_TIMER12_IRQHandler          /* TIM8 Update and TIM13        */
  .word     TIMER7_TRG_CMT_TIMER13_IRQHandler     /* TIM8 Trigger and Commutation and TIM14 */
  .word     TIMER7_Channel_IRQHandler                /* TIM8 Capture Compare         */
  .word     DMA0_Channel7_IRQHandler           /* DMA1 Stream7                 */
  .word     EXMC_IRQHandler                    /* FMC                          */
  .word     SDIO_IRQHandler                   /* SDIO                         */
  .word     TIMER4_IRQHandler                   /* TIM5                         */
  .word     SPI2_IRQHandler                   /* SPI3                         */
  .word     UART3_IRQHandler                  /* UART4                        */
  .word     UART4_IRQHandler                  /* UART5                        */
  .word     TIMER5_DAC_IRQHandler               /* TIM6 and DAC1&2 underrun errors */
  .word     TIMER6_IRQHandler                   /* TIM7                         */
  .word     DMA1_Channel0_IRQHandler           /* DMA2 Stream 0                */
  .word     DMA1_Channel1_IRQHandler           /* DMA2 Stream 1                */
  .word     DMA1_Channel2_IRQHandler           /* DMA2 Stream 2                */
  .word     DMA1_Channel3_IRQHandler           /* DMA2 Stream 3                */
  .word     DMA1_Channel4_IRQHandler           /* DMA2 Stream 4                */
  .word     ENET_IRQHandler                    /* Ethernet                     */
  .word     ENET_WKUP_IRQHandler               /* Ethernet Wakeup through EXTI line */
  .word     CAN1_TX_IRQHandler                /* CAN2 TX                      */
  .word     CAN1_RX0_IRQHandler               /* CAN2 RX0                     */
  .word     CAN1_RX1_IRQHandler               /* CAN2 RX1                     */
  .word     CAN1_EWMC_IRQHandler               /* CAN2 SCE                     */
  .word     USBFS_IRQHandler                 /* USB OTG FS                   */
  .word     DMA1_Channel5_IRQHandler           /* DMA2 Stream 5                */
  .word     DMA1_Channel6_IRQHandler           /* DMA2 Stream 6                */
  .word     DMA1_Channel7_IRQHandler           /* DMA2 Stream 7                */
  .word     USART5_IRQHandler                 /* USART6                       */
  .word     I2C2_EV_IRQHandler                /* I2C3 event                   */
  .word     I2C2_ER_IRQHandler                /* I2C3 error                   */
  .word     USBHS_EP1_Out_IRQHandler         /* USB OTG HS End Point 1 Out   */
  .word     USBHS_EP1_In_IRQHandler          /* USB OTG HS End Point 1 In    */
  .word     USBHS_WKUP_IRQHandler            /* USB OTG HS Wakeup through EXTI */
  .word     USBHS_IRQHandler                 /* USB OTG HS                   */
  .word     DCI_IRQHandler                   /* DCMI                         */
  .word     0                                /* CRYP crypto                  */
  .word     TRNG_IRQHandler                  /* Hash and Rng                 */
  .word     FPU_IRQHandler                    /* FPU                          */
  .word     UART6_IRQHandler                  /* UART7                        */
  .word     UART7_IRQHandler                  /* UART8                        */
  .word     SPI3_IRQHandler                   /* SPI4                         */
  .word     SPI4_IRQHandler                   /* SPI5 						  */
  .word     SPI5_IRQHandler                   /* SPI6						  */
  .word     0                                /* SAI1						  */
  .word     TLI_IRQHandler                    /* LTDC           		      */
  .word     TLI_ER_IRQHandler                 /* LTDC error          	      */
  .word     IPA_IRQHandler                  /* DMA2D                        */



/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/
   .weak      NMI_Handler
   .thumb_set NMI_Handler,Default_Handler

   .weak      HardFault_Handler
   .thumb_set HardFault_Handler,Default_Handler

   .weak      MemManage_Handler
   .thumb_set MemManage_Handler,Default_Handler

   .weak      BusFault_Handler
   .thumb_set BusFault_Handler,Default_Handler

   .weak      UsageFault_Handler
   .thumb_set UsageFault_Handler,Default_Handler

   .weak      SVC_Handler
   .thumb_set SVC_Handler,Default_Handler

   .weak      DebugMon_Handler
   .thumb_set DebugMon_Handler,Default_Handler

   .weak      PendSV_Handler
   .thumb_set PendSV_Handler,Default_Handler

   .weak      SysTick_Handler
   .thumb_set SysTick_Handler,Default_Handler

   .weak      WWDGT_IRQHandler
   .thumb_set WWDGT_IRQHandler,Default_Handler

   .weak      LVD_IRQHandler
   .thumb_set LVD_IRQHandler,Default_Handler

   .weak      TAMPER_STAMP_IRQHandler
   .thumb_set TAMPER_STAMP_IRQHandler,Default_Handler

   .weak      RTC_WKUP_IRQHandler
   .thumb_set RTC_WKUP_IRQHandler,Default_Handler

   .weak      FMC_IRQHandler
   .thumb_set FMC_IRQHandler,Default_Handler

   .weak      RCU_CTC_IRQHandler
   .thumb_set RCU_CTC_IRQHandler,Default_Handler

   .weak      EXTI0_IRQHandler
   .thumb_set EXTI0_IRQHandler,Default_Handler

   .weak      EXTI1_IRQHandler
   .thumb_set EXTI1_IRQHandler,Default_Handler

   .weak      EXTI2_IRQHandler
   .thumb_set EXTI2_IRQHandler,Default_Handler

   .weak      EXTI3_IRQHandler
   .thumb_set EXTI3_IRQHandler,Default_Handler

   .weak      EXTI4_IRQHandler
   .thumb_set EXTI4_IRQHandler,Default_Handler

   .weak      DMA0_Channel0_IRQHandler
   .thumb_set DMA0_Channel0_IRQHandler,Default_Handler

   .weak      DMA0_Channel1_IRQHandler
   .thumb_set DMA0_Channel1_IRQHandler,Default_Handler

   .weak      DMA0_Channel2_IRQHandler
   .thumb_set DMA0_Channel2_IRQHandler,Default_Handler

   .weak      DMA0_Channel3_IRQHandler
   .thumb_set DMA0_Channel3_IRQHandler,Default_Handler

   .weak      DMA0_Channel4_IRQHandler
   .thumb_set DMA0_Channel4_IRQHandler,Default_Handler

   .weak      DMA0_Channel5_IRQHandler
   .thumb_set DMA0_Channel5_IRQHandler,Default_Handler

   .weak      DMA0_Channel6_IRQHandler
   .thumb_set DMA0_Channel6_IRQHandler,Default_Handler

   .weak      ADC_IRQHandler
   .thumb_set ADC_IRQHandler,Default_Handler

   .weak      CAN0_TX_IRQHandler
   .thumb_set CAN0_TX_IRQHandler,Default_Handler

   .weak      CAN0_RX0_IRQHandler
   .thumb_set CAN0_RX0_IRQHandler,Default_Handler

   .weak      CAN0_RX1_IRQHandler
   .thumb_set CAN0_RX1_IRQHandler,Default_Handler

   .weak      CAN0_EWMC_IRQHandler
   .thumb_set CAN0_EWMC_IRQHandler,Default_Handler

   .weak      EXTI5_9_IRQHandler
   .thumb_set EXTI5_9_IRQHandler,Default_Handler

   .weak      TIMER0_BRK_TIMER8_IRQHandler
   .thumb_set TIMER0_BRK_TIMER8_IRQHandler,Default_Handler

   .weak      TIMER0_UP_TIMER9_IRQHandler
   .thumb_set TIMER0_UP_TIMER9_IRQHandler,Default_Handler

   .weak      TIMER0_TRG_CMT_TIMER10_IRQHandler
   .thumb_set TIMER0_TRG_CMT_TIMER10_IRQHandler,Default_Handler

   .weak      TIMER0_Channel_IRQHandler
   .thumb_set TIMER0_Channel_IRQHandler,Default_Handler

   .weak      TIMER1_IRQHandler
   .thumb_set TIMER1_IRQHandler,Default_Handler

   .weak      TIMER2_IRQHandler
   .thumb_set TIMER2_IRQHandler,Default_Handler

   .weak      TIMER3_IRQHandler
   .thumb_set TIMER3_IRQHandler,Default_Handler

   .weak      I2C0_EV_IRQHandler
   .thumb_set I2C0_EV_IRQHandler,Default_Handler

   .weak      I2C0_ER_IRQHandler
   .thumb_set I2C0_ER_IRQHandler,Default_Handler

   .weak      I2C1_EV_IRQHandler
   .thumb_set I2C1_EV_IRQHandler,Default_Handler

   .weak      I2C1_ER_IRQHandler
   .thumb_set I2C1_ER_IRQHandler,Default_Handler

   .weak      SPI0_IRQHandler
   .thumb_set SPI0_IRQHandler,Default_Handler

   .weak      SPI1_IRQHandler
   .thumb_set SPI1_IRQHandler,Default_Handler

   .weak      USART0_IRQHandler
   .thumb_set USART0_IRQHandler,Default_Handler

   .weak      USART1_IRQHandler
   .thumb_set USART1_IRQHandler,Default_Handler

   .weak      USART2_IRQHandler
   .thumb_set USART2_IRQHandler,Default_Handler

   .weak      EXTI10_15_IRQHandler
   .thumb_set EXTI10_15_IRQHandler,Default_Handler

   .weak      RTC_Alarm_IRQHandler
   .thumb_set RTC_Alarm_IRQHandler,Default_Handler

   .weak      USBFS_WKUP_IRQHandler
   .thumb_set USBFS_WKUP_IRQHandler,Default_Handler

   .weak      TIMER7_BRK_TIMER11_IRQHandler
   .thumb_set TIMER7_BRK_TIMER11_IRQHandler,Default_Handler

   .weak      TIMER7_UP_TIMER12_IRQHandler
   .thumb_set TIMER7_UP_TIMER12_IRQHandler,Default_Handler

   .weak      TIMER7_TRG_CMT_TIMER13_IRQHandler
   .thumb_set TIMER7_TRG_CMT_TIMER13_IRQHandler,Default_Handler

   .weak      TIMER7_Channel_IRQHandler
   .thumb_set TIMER7_Channel_IRQHandler,Default_Handler

   .weak      DMA0_Channel7_IRQHandler
   .thumb_set DMA0_Channel7_IRQHandler,Default_Handler

   .weak      EXMC_IRQHandler
   .thumb_set EXMC_IRQHandler,Default_Handler

   .weak      SDIO_IRQHandler
   .thumb_set SDIO_IRQHandler,Default_Handler

   .weak      TIMER4_IRQHandler
   .thumb_set TIMER4_IRQHandler,Default_Handler

   .weak      SPI2_IRQHandler
   .thumb_set SPI2_IRQHandler,Default_Handler

   .weak      UART3_IRQHandler
   .thumb_set UART3_IRQHandler,Default_Handler

   .weak      UART4_IRQHandler
   .thumb_set UART4_IRQHandler,Default_Handler

   .weak      TIMER5_DAC_IRQHandler
   .thumb_set TIMER5_DAC_IRQHandler,Default_Handler

   .weak      TIMER6_IRQHandler
   .thumb_set TIMER6_IRQHandler,Default_Handler

   .weak      DMA1_Channel0_IRQHandler
   .thumb_set DMA1_Channel0_IRQHandler,Default_Handler

   .weak      DMA1_Channel1_IRQHandler
   .thumb_set DMA1_Channel1_IRQHandler,Default_Handler

   .weak      DMA1_Channel2_IRQHandler
   .thumb_set DMA1_Channel2_IRQHandler,Default_Handler

   .weak      DMA1_Channel3_IRQHandler
   .thumb_set DMA1_Channel3_IRQHandler,Default_Handler

   .weak      DMA1_Channel4_IRQHandler
   .thumb_set DMA1_Channel4_IRQHandler,Default_Handler

   .weak      ENET_IRQHandler
   .thumb_set ENET_IRQHandler,Default_Handler

   .weak      ENET_WKUP_IRQHandler
   .thumb_set ENET_WKUP_IRQHandler,Default_Handler

   .weak      CAN1_TX_IRQHandler
   .thumb_set CAN1_TX_IRQHandler,Default_Handler

   .weak      CAN1_RX0_IRQHandler
   .thumb_set CAN1_RX0_IRQHandler,Default_Handler

   .weak      CAN1_RX1_IRQHandler
   .thumb_set CAN1_RX1_IRQHandler,Default_Handler

   .weak      CAN1_EWMC_IRQHandler
   .thumb_set CAN1_EWMC_IRQHandler,Default_Handler

   .weak      USBFS_IRQHandler
   .thumb_set USBFS_IRQHandler,Default_Handler

   .weak      DMA1_Channel5_IRQHandler
   .thumb_set DMA1_Channel5_IRQHandler,Default_Handler

   .weak      DMA1_Channel6_IRQHandler
   .thumb_set DMA1_Channel6_IRQHandler,Default_Handler

   .weak      DMA1_Channel7_IRQHandler
   .thumb_set DMA1_Channel7_IRQHandler,Default_Handler

   .weak      USART5_IRQHandler
   .thumb_set USART5_IRQHandler,Default_Handler

   .weak      I2C2_EV_IRQHandler
   .thumb_set I2C2_EV_IRQHandler,Default_Handler

   .weak      I2C2_ER_IRQHandler
   .thumb_set I2C2_ER_IRQHandler,Default_Handler

   .weak      USBHS_EP1_Out_IRQHandler
   .thumb_set USBHS_EP1_Out_IRQHandler,Default_Handler

   .weak      USBHS_EP1_In_IRQHandler
   .thumb_set USBHS_EP1_In_IRQHandler,Default_Handler

   .weak      USBHS_WKUP_IRQHandler
   .thumb_set USBHS_WKUP_IRQHandler,Default_Handler

   .weak      USBHS_IRQHandler
   .thumb_set USBHS_IRQHandler,Default_Handler

   .weak      DCI_IRQHandler
   .thumb_set DCI_IRQHandler,Default_Handler

   .weak      TRNG_IRQHandler
   .thumb_set TRNG_IRQHandler,Default_Handler

   .weak      FPU_IRQHandler
   .thumb_set FPU_IRQHandler,Default_Handler

   .weak      UART6_IRQHandler
   .thumb_set UART6_IRQHandler,Default_Handler

   .weak      UART7_IRQHandler
   .thumb_set UART7_IRQHandler,Default_Handler

   .weak      SPI3_IRQHandler
   .thumb_set SPI3_IRQHandler,Default_Handler

   .weak      SPI4_IRQHandler
   .thumb_set SPI4_IRQHandler,Default_Handler

   .weak      SPI5_IRQHandler
   .thumb_set SPI5_IRQHandler,Default_Handler

   .weak      TLI_IRQHandler
   .thumb_set TLI_IRQHandler,Default_Handler

   .weak      TLI_ER_IRQHandler
   .thumb_set TLI_ER_IRQHandler,Default_Handler

   .weak      IPA_IRQHandler
   .thumb_set IPA_IRQHandler,Default_Handler
