/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Netconn_RTOS/Src/app_ethernet.c 
  * @author  MCD Application Team
  * @brief   Ethernet specefic module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "app_ethernet.h"
#include "main.h"
#include "lwip.h"
#include "cmsis_os.h"
#include "lwip/opt.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"

#include "ethernetif.h"
#include "string.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**


  * @brief  Notify the User about the network interface config status 
  * @param  netif: the network interface
  * @retval None
  */
void ethernet_link_status_updated(struct netif *netif) 
{
  if(!(netif_is_up(netif)))
  {
	  printf("ETH Link Down\n");

	  //
	  //while link is down the system should wait, i should set a hw semaphore to halt cm7?


  }
  else {
	  int local_IP = netif->ip_addr.addr;
  //if ethernet link is up, turn off red led
	  printf("ETH Link up. IP: %d.%d.%d.%d\n",(local_IP & 0xff), ((local_IP >> 8) & 0xff), ((local_IP >> 16) & 0xff), (local_IP >> 24));

  }
}


