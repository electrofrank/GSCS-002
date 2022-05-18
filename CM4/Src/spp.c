/*
 * spp.c

 *
 *  Created on: May 18, 2022
 *      Author: Francesco
 */
#include "spp.h"
#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#if defined ( __CC_ARM )  /* MDK ARM Compiler */
#include "lwip/sio.h"
#endif /* MDK ARM Compiler */
#include "ethernetif.h"
#include <string.h>

/* USER CODE BEGIN 0 */
#include "lwip/udp.h"

struct udp_pcb *upcb;
struct udp_pcb *upcb2;

char UDP_RX_BUFFER[50];
char UDP_TX_BUFFER[50];

uint8_t SPP_TC_len = 0;
uint8_t SPP_NEW_TC = 0;

uint8_t SPP_TM_len = 0;

struct netif gnetif;

ip4_addr_t ipaddr;
ip4_addr_t netmask;
ip4_addr_t gw;
uint8_t IP_ADDRESS[4];
uint8_t NETMASK_ADDRESS[4];
uint8_t GATEWAY_ADDRESS[4];

ip_addr_t SERVER_ipaddr;

void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p,
		const ip_addr_t *addr, u16_t port) {

	//Verify the data dimension
	int RX_COMMAND_LENGHT = (int) p->len;

	if (RX_COMMAND_LENGHT == 8)
		/* Copy the data from the pbuf */
		strncpy(UDP_RX_BUFFER, (char*) p->payload, 8);

	else {
		printf("Command Lenght Error : %d \n", RX_COMMAND_LENGHT);
	}
	/* Free receive pbuf */

	pbuf_free(p);
}

void udpClient_send_spp() {

	struct pbuf *txBuf;

	/* allocate pbuf from pool*/
	txBuf = pbuf_alloc(PBUF_TRANSPORT, SPP_TM_len, PBUF_RAM);

	if (txBuf != NULL) {
		/* copy data to pbuf */
		pbuf_take(txBuf, &UDP_TX_BUFFER, SPP_TM_len);

		/* send udp data */
		if ((udp_send(upcb, txBuf)) == ERR_OK) {
		} else
			printf("Error Sending UDP Packet\n");

		/* free pbuf */
		pbuf_free(txBuf);
	}
}

void udpClient_connect(void) {

	//SERVER STATIC IP
	IP_ADDR4(&SERVER_ipaddr, 192, 168, 1, 100);
	//IP_ADDR4(&SERVER_ipaddr, 192, 168, 1, 103);

	/* 1. Create a new UDP control block  */
	upcb = udp_new();

	/* Bind the block to module's IP and port */

	if (udp_bind(upcb, &ipaddr, 50000) == ERR_USE)
		printf("Error binding UDP1\n");
	else
		//printf("UDP module bind on Port 50000 for Telemetry TX\n");

	/* configure destination IP address and port for telemetry server */
	udp_connect(upcb, &SERVER_ipaddr, 10015);

	//register udp receiver callback on upcb2
	udp_recv(upcb, udp_receive_callback, NULL);

}


void SPP_TM_hdr_config(primaryHdr TM_hdr) {

	TM_hdr.APID = 5;
	TM_hdr.pkt_ver = 0;
	TM_hdr.secHdrFlag = 0;
	TM_hdr.type = 0;
	TM_hdr.data_len = SPP_BYTE_DATA; //bytes in sec. header + data minus one (0 + 6) - 1
	TM_hdr.seqFlag = 3;
}

void unpack_SPP_TC() {

	if (SPP_NEW_TC) {
		SPP_NEW_TC = 0;
	primaryHdr TC_pck_hdr;
	space_packet TC_pck;

	TC_pck_hdr.pkt_ver = (uint8_t)(UDP_RX_BUFFER[0] >> 5);
	TC_pck_hdr.type = (uint8_t)(UDP_RX_BUFFER[0] >> 4); //version is alwais 000
	TC_pck_hdr.secHdrFlag = 0;
	TC_pck_hdr.APID = UDP_RX_BUFFER[1];   //apid limited to 254 ((bfr[0] & 7)<<8)|(bfr[1]);
	TC_pck_hdr.seqFlag = UDP_RX_BUFFER[2] >> 6;   //apid limited to 254 ((bfr[0] & 7)<<8)|(bfr[1]);
	TC_pck_hdr.seqCount = ((UDP_RX_BUFFER[2] & 0x3F)<<6)|UDP_RX_BUFFER[3];
	TC_pck_hdr.data_len = UDP_RX_BUFFER[4] << 8 | UDP_RX_BUFFER[5];

/*  printf("\nUnpacking SPP TC packet...\n");

	for(int k = 0; k < SPP_len; k++) {
		printf("%x ",UDP_RX_BUFFER[k]); //print byte in hex
	}
	printf("\n");

	printf("SPP Version: %d\n",TC_pck_hdr.pkt_ver);
	printf(TC_pck_hdr.type == 1 ? "SPP type: TC\n" : "SPP type: TM\n");
	printf("SPP APID: %d\n",TC_pck_hdr.APID);
	printf("SPP Seq flag: %d\n",TC_pck_hdr.seqFlag);
	printf("SPP Seq count: %d\n",TC_pck_hdr.seqCount);
	printf("SPP data len: %d\n",TC_pck_hdr.data_len);

*/

	//fill spp primary header
	TC_pck.pHdr = TC_pck_hdr;

	for(int k = 0; k < (TC_pck_hdr.data_len)+1; k++) {

		TC_pck.data[k] = UDP_RX_BUFFER[k+6]; //data are made of 16 bit

		//printf("SPP data%d: %d\n",k,TC_pck.data[k]);
		}


	parse_SPP_TC(TC_pck); //execute command actions

	}

	else {}

}

void pack_SPP_TM(space_packet tm_pkt) {

	//take data from space packet struct and put it in an ordered char buffer
	 //    TM_PKT --> UDP_TX_BUFFER
	 //calculate packet lenght from datalenght -> SPP_TM_len
	 SPP_TM_len = tm_pkt.pHdr.data_len + 6;// total lenght is size of data + size of primary hdr

	 //copy pkt hdr into udp tx buffer
	 UDP_TX_BUFFER[0] = 0;
	 UDP_TX_BUFFER[1] = 5;
	 UDP_TX_BUFFER[2] = (3 << 6 | (tm_pkt.pHdr.seqCount >> 4));
	 UDP_TX_BUFFER[3] =  (tm_pkt.pHdr.seqCount) << 8;
	 UDP_TX_BUFFER[4] = 0;
	 UDP_TX_BUFFER[5] = tm_pkt.pHdr.data_len;

	 for(int j = 6; j < tm_pkt.pHdr.data_len + 6; j++) {
		 UDP_TX_BUFFER[j] = tm_pkt.data[j - 6];
	 }
}

void parse_SPP_TC(space_packet pkt) {

//execute action depending on the SPP packet APID and content
//////////////////// IGNITION COMMAND /////////////////////////////

if (((pkt.data[0] << 8) | pkt.data[1]) == 200) {
		HAL_GPIO_WritePin(GPIOD,IGN_OUT_Pin, SET);
		osDelay(2000);
		HAL_GPIO_WritePin(GPIOD,IGN_OUT_Pin, RESET);
	}
else HAL_GPIO_WritePin(GPIOD,IGN_OUT_Pin, RESET);

//GAS SOLENOID 1 ON COMMAND
if (((pkt.data[0] << 8) | pkt.data[1]) == 207) {}
	//HAL_GPIO_WritePin(WSS_SSR4_GPIO_Port, WSS_SSR4_Pin, SET);

//GAS SOLENOID 1 OFF COMMAND
if (((pkt.data[0] << 8) | pkt.data[1]) == 208) {}
	//HAL_GPIO_WritePin(WSS_SSR4_GPIO_Port, WSS_SSR4_Pin, RESET);

//GAS SOLENOID 2 ON COMMAND
if (((pkt.data[0] << 8) | pkt.data[1]) == 209) {}
	//HAL_GPIO_WritePin(WSS_SSR4_GPIO_Port, WSS_SSR4_Pin, SET);

//GAS SOLENOID 2 OFF COMMAND
if (((pkt.data[0] << 8) | pkt.data[1]) == 210) {}
	//HAL_GPIO_WritePin(WSS_SSR4_GPIO_Port, WSS_SSR4_Pin, RESET);



}
