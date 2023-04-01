/*!
    \file    enet_cfg.c
    \brief   Ethernet configuration module
*/
#include "gd32f4xx.h"
#include <stdio.h>
#include <string.h>
#include "systick.h"
#include "gd32f4xx_enet.h"

/* ENET RxDMA/TxDMA descriptor */
extern enet_descriptors_struct  rxdesc_tab[ENET_RXBUF_NUM], txdesc_tab[ENET_TXBUF_NUM];

/* ENET receive buffer  */
extern uint8_t rx_buff[ENET_RXBUF_NUM][ENET_RXBUF_SIZE];

/* ENET transmit buffer */
extern uint8_t tx_buff[ENET_TXBUF_NUM][ENET_TXBUF_SIZE];

/*global transmit and receive descriptors pointers */
extern enet_descriptors_struct  *dma_current_txdesc;
extern enet_descriptors_struct  *dma_current_rxdesc;

/* preserve another ENET RxDMA/TxDMA ptp descriptor for normal mode */
enet_descriptors_struct  ptp_txstructure[ENET_TXBUF_NUM];
enet_descriptors_struct  ptp_rxstructure[ENET_RXBUF_NUM];

uint8_t enet_hwaddr[6] = {00, 0xED, 0x20, 0x03, 0x02, 0x3};

uint8_t enet_tx_packet[64] = {
		   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xED,
		   0x20, 0x03, 0x02, 0x01, 0x08, 0x00, 0x45, 0x00,
		   0x00, 0x34, 0x00, 0x00, 0x40, 0x00, 0x40, 0x11,
		   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static __IO uint32_t enet_init_status = 0;

void enet_gpio_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOE);

    /* enable SYSCFG clock */
    rcu_periph_clock_enable(RCU_SYSCFG);

    syscfg_enet_phy_interface_config(SYSCFG_ENET_PHY_MII);

    /* PA1: ETH_MII_RX_CLK+ */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_1);

    /* PA2: ETH_MDIO+ */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_2);

    /* PA3: ETH_MII_COL+ */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_3);

    /* PA7: ETH_MII_RX_DV+ */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_7);

#ifdef ETH_CRS_WKP_PIN_USED
    /* PA0: ETH_MII_CRS */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_0);

    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_0);
#endif

    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_1);
    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_2);
    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_3);
    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_7);

    /* PE2: ETH_MII_TXD3+ */
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_2);

    gpio_af_set(GPIOE, GPIO_AF_11, GPIO_PIN_2);

    /* PB0: ETH_MII_RXD2+ */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_0);

    /* PB1: ETH_MII_RXD3+ */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_1);

    /* PB10: ETH_MII_RX_ER+ */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_10);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_10);

    /* PB11: ETH_MII_TX_EN+ */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_11);

    /* PB12: ETH_MII_TXD0+ */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_12);

    /* PB13: ETH_MII_TXD1+ */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_13);

    gpio_af_set(GPIOB, GPIO_AF_11, GPIO_PIN_0);
    gpio_af_set(GPIOB, GPIO_AF_11, GPIO_PIN_1);
    gpio_af_set(GPIOB, GPIO_AF_11, GPIO_PIN_10);
    gpio_af_set(GPIOB, GPIO_AF_11, GPIO_PIN_11);
    gpio_af_set(GPIOB, GPIO_AF_11, GPIO_PIN_12);
    gpio_af_set(GPIOB, GPIO_AF_11, GPIO_PIN_13);

    /* PC1: ETH_MDC+ */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_1);

    /* PC2: ETH_MII_TXD2+ */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_2);

    /* PC3: ETH_MII_TX_CLK+ */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_3);

    /* PC4: ETH_MII_RXD0+ */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_4);

    /* PC5: ETH_MII_RXD1+ */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_5);

    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_1);
    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_2);
    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_3);
    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_4);
    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_5);
}

#ifdef USE_ENET_INTERRUPT
/*!
    \brief      configures the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void nvic_configuration(void)
{
    //nvic_vector_table_set(NVIC_VECTTAB_FLASH, 0x0);
    //nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    nvic_irq_enable(ENET_IRQn, 0, 0);
}
#endif /* USE_ENET_INTERRUPT */

/*!
    \brief      configures the ethernet interface
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void enet_mac_dma_config(void)
{
    ErrStatus reval_state = ERROR;

    /* enable ethernet clock  */
    rcu_periph_clock_enable(RCU_ENET);
    rcu_periph_clock_enable(RCU_ENETTX);
    rcu_periph_clock_enable(RCU_ENETRX);

    /* reset ethernet on AHB bus */
    enet_deinit();

    reval_state = enet_software_reset();
    if(ERROR == reval_state) {
        while(1) {}
    }

    /* configure the parameters which are usually less cared for enet initialization */
//  enet_initpara_config(HALFDUPLEX_OPTION, ENET_CARRIERSENSE_ENABLE|ENET_RECEIVEOWN_ENABLE|ENET_RETRYTRANSMISSION_DISABLE|ENET_BACKOFFLIMIT_10|ENET_DEFERRALCHECK_DISABLE);
//  enet_initpara_config(DMA_OPTION, ENET_FLUSH_RXFRAME_ENABLE|ENET_SECONDFRAME_OPT_ENABLE|ENET_NORMAL_DESCRIPTOR);

#ifdef CHECKSUM_BY_HARDWARE
    enet_init_status = enet_init(ENET_AUTO_NEGOTIATION, ENET_AUTOCHECKSUM_DROP_FAILFRAMES, ENET_BROADCAST_FRAMES_PASS);
#else
    enet_init_status = enet_init(ENET_AUTO_NEGOTIATION, ENET_NO_AUTOCHECKSUM, ENET_BROADCAST_FRAMES_PASS);
#endif /* CHECKSUM_BY_HARDWARE */

}

/*!
    \brief      setup ethernet system(GPIOs, clocks, MAC, DMA, systick)
    \param[in]  none
    \param[out] none
    \retval     none
*/
void enet_system_setup(void)
{
	printf("ENET init\r\n");

#ifdef USE_ENET_INTERRUPT
    nvic_configuration();
#endif /* USE_ENET_INTERRUPT */

    /* configure the GPIO ports for ethernet pins */
    enet_gpio_init();

    /* configure the ethernet MAC/DMA */
    enet_mac_dma_config();

    if(0 == enet_init_status) {
        while(1) {
        }
    }

#ifdef USE_ENET_INTERRUPT
    enet_interrupt_enable(ENET_DMA_INT_NIE);
    enet_interrupt_enable(ENET_DMA_INT_RIE);
#endif /* USE_ENET_INTERRUPT */

#ifdef SELECT_DESCRIPTORS_ENHANCED_MODE
    enet_desc_select_enhanced_mode();
#endif /* SELECT_DESCRIPTORS_ENHANCED_MODE */

    /* initialize MAC address in ethernet MAC */
    enet_mac_address_set(ENET_MAC_ADDRESS0, enet_hwaddr);


    /* initialize descriptors list: chain/ring mode */
#ifdef SELECT_DESCRIPTORS_ENHANCED_MODE
    enet_ptp_enhanced_descriptors_chain_init(ENET_DMA_TX);
    enet_ptp_enhanced_descriptors_chain_init(ENET_DMA_RX);
#else

    enet_descriptors_chain_init(ENET_DMA_TX);
    enet_descriptors_chain_init(ENET_DMA_RX);

//    enet_descriptors_ring_init(ENET_DMA_TX);
//    enet_descriptors_ring_init(ENET_DMA_RX);

#endif /* SELECT_DESCRIPTORS_ENHANCED_MODE */

    /* enable ethernet Rx interrrupt */
    {   int i;
        for(i=0; i<ENET_RXBUF_NUM; i++){
           enet_rx_desc_immediate_receive_complete_interrupt(&rxdesc_tab[i]);
        }
    }

#ifdef CHECKSUM_BY_HARDWARE
    /* enable the TCP, UDP and ICMP checksum insertion for the Tx frames */
    for(i=0; i < ENET_TXBUF_NUM; i++){
        enet_transmit_checksum_config(&txdesc_tab[i], ENET_CHECKSUM_TCPUDPICMP_FULL);
    }
#endif /* CHECKSUM_BY_HARDWARE */

    /* note: TCP, UDP, ICMP checksum checking for received frame are enabled in DMA config */

    /* enable MAC and DMA transmission and reception */
    enet_enable();

    printf("ENET OK\r\n");
}

/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
uint16_t enet_rx_frame(void)
{
	uint16_t len;
    uint8_t *buffer;

    /* obtain the size of the packet and put it into the "len" variable. */
    len = enet_desc_information_get(dma_current_rxdesc, RXDESC_FRAME_LENGTH);
    buffer = (uint8_t *)(enet_desc_information_get(dma_current_rxdesc, RXDESC_BUFFER_1_ADDR));
    (void)buffer;

#ifdef SELECT_DESCRIPTORS_ENHANCED_MODE
    ENET_NOCOPY_PTPFRAME_RECEIVE_ENHANCED_MODE(NULL);

#else

    ENET_NOCOPY_FRAME_RECEIVE();
#endif /* SELECT_DESCRIPTORS_ENHANCED_MODE */

    return len;
}

void enet_tx_test_frame(void)
{
    int framelength = 0;
    uint8_t *buffer;

    while((uint32_t)RESET != (dma_current_txdesc->status & ENET_TDES0_DAV)){
    }
    buffer = (uint8_t *)(enet_desc_information_get(dma_current_txdesc, TXDESC_BUFFER_1_ADDR));

    /* copy frame from pbufs to driver buffers */
    framelength = 64;
    memcpy(buffer, enet_tx_packet, 64);

    /* note: padding and CRC for transmitted frame
       are automatically inserted by DMA */

    /* transmit descriptors to give to DMA */
#ifdef SELECT_DESCRIPTORS_ENHANCED_MODE
    ENET_NOCOPY_PTPFRAME_TRANSMIT_ENHANCED_MODE(framelength, NULL);

#else

    ENET_NOCOPY_FRAME_TRANSMIT(framelength);
#endif /* SELECT_DESCRIPTORS_ENHANCED_MODE */
}
