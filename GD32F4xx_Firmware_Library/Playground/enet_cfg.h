/*!
    \file    enet_cfg.c
    \brief   Header for Ethernet configuration module
*/

#ifndef ENET_CFG__H
#define ENET_CFG__H


void enet_system_setup(void);
uint16_t enet_rx_frame(void);
void enet_tx_test_frame(void);

#endif /* ENET_CFG__H */
