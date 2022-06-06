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
#include "main.h"
#include "hx711.h"

#define UDP_TM_PORT 10015 // TELEMETRY
#define UDP_TC_PORT 10025 // TELECOMMAND

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

hx711_t LOAD_CELL1; //LC1
hx711_t LOAD_CELL2; //LC2
hx711_t LOAD_CELL3; //LC3
hx711_t LOAD_CELL4; //LC4

void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p,
		const ip_addr_t *addr, u16_t port) {

	/* Copy the data from the pbuf */ //
	memcpy(UDP_RX_BUFFER, (char*)p->payload,p->len);

	SPP_TC_len = (int) p->len;
	SPP_NEW_TC = 1; //

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
	upcb2 = udp_new();

		/* Bind to local IP and local port */

		if (udp_bind(upcb, IP_ANY_TYPE, 5005) == ERR_USE)
			printf("Error binding UDP\n");
		else
			printf("UDP bind on Port %d for TM trasmission \n", 5005);

		/* configure destination IP address and port for telemetry server */
		udp_connect(upcb, &SERVER_ipaddr, UDP_TM_PORT);

		udp_bind(upcb2, IP_ANY_TYPE, UDP_TC_PORT); //port to receive TC

		//register udp receiver callback on upcb2
		udp_recv(upcb2, udp_receive_callback, NULL);

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
		SPP_NEW_TC = 0; //reset new packet flag

	primaryHdr TC_pck_hdr;
	space_packet TC_pck;

	TC_pck_hdr.pkt_ver = (uint8_t)(UDP_RX_BUFFER[0] >> 5);
	TC_pck_hdr.type = (uint8_t)(UDP_RX_BUFFER[0] >> 4); //version is alwais 000
	TC_pck_hdr.secHdrFlag = 0;
	TC_pck_hdr.APID = UDP_RX_BUFFER[1];   //apid limited to 254 ((bfr[0] & 7)<<8)|(bfr[1]);
	TC_pck_hdr.seqFlag = UDP_RX_BUFFER[2] >> 6;   //apid limited to 254 ((bfr[0] & 7)<<8)|(bfr[1]);
	TC_pck_hdr.seqCount = ((UDP_RX_BUFFER[2] & 0x3F)<<6)|UDP_RX_BUFFER[3];
	TC_pck_hdr.data_len = UDP_RX_BUFFER[4] << 8 | UDP_RX_BUFFER[5];

//	printf("\nUnpacking SPP TC packet...\n");
//
//	for(int k = 0; k < SPP_TC_len; k++) {
//		printf("%x ",UDP_RX_BUFFER[k]); //print byte in hex
//	}
//	printf("\n");
//
//	printf("SPP Version: %d\n",TC_pck_hdr.pkt_ver);
//	printf(TC_pck_hdr.type == 1 ? "SPP type: TC\n" : "SPP type: TM\n");
//	printf("SPP APID: %d\n",TC_pck_hdr.APID);
//	printf("SPP Seq flag: %d\n",TC_pck_hdr.seqFlag);
//	printf("SPP Seq count: %d\n",TC_pck_hdr.seqCount);
//	printf("SPP data len: %d\n",TC_pck_hdr.data_len);



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
//printf("Parsing TC\n");

if (((pkt.data[0] << 8) | pkt.data[1]) == 200) {
		HAL_GPIO_WritePin(GPIOD,IGN_OUT_Pin, SET);
		osDelay(2000); //blocking, not good
		HAL_GPIO_WritePin(GPIOD,IGN_OUT_Pin, RESET);
	}
else HAL_GPIO_WritePin(GPIOD,IGN_OUT_Pin, RESET);

//GAS SOLENOID 1 ON COMMAND
if (((pkt.data[0] << 8) | pkt.data[1]) == 207)
	HAL_GPIO_WritePin(GPIOE, LOW_SIDE_SW_2_Pin ,GPIO_PIN_SET);//solenoid 1 on command

//GAS SOLENOID 1 OFF COMMAND
if (((pkt.data[0] << 8) | pkt.data[1]) == 208)
	HAL_GPIO_WritePin(GPIOE, LOW_SIDE_SW_2_Pin ,GPIO_PIN_RESET);

//GAS SOLENOID 2 ON COMMAND
if (((pkt.data[0] << 8) | pkt.data[1]) == 209)
	HAL_GPIO_WritePin(GPIOE, LOW_SIDE_SW_1_Pin ,GPIO_PIN_SET);

//GAS SOLENOID 2 OFF COMMAND
if (((pkt.data[0] << 8) | pkt.data[1]) == 210)
	HAL_GPIO_WritePin(GPIOE, LOW_SIDE_SW_1_Pin ,GPIO_PIN_RESET);


//START STATIC TEST FIRE (IGNITION + VALVE 1 AND VALVE 2)
if (((pkt.data[0] << 8) | pkt.data[1]) == 26) {

	HAL_GPIO_WritePin(GPIOD,IGN_OUT_Pin, SET);
	HAL_GPIO_WritePin(GPIOE, LOW_SIDE_SW_2_Pin ,GPIO_PIN_SET);//solenoid 1 on command
	HAL_GPIO_WritePin(GPIOE, LOW_SIDE_SW_1_Pin ,GPIO_PIN_SET);//solenoid 2 on command
	osDelay(2000); //blocking, not good
	HAL_GPIO_WritePin(GPIOD,IGN_OUT_Pin, RESET);
}

if (((pkt.data[0] << 8) | pkt.data[1]) == 27) {

	HAL_GPIO_WritePin(GPIOE, LOW_SIDE_SW_2_Pin ,GPIO_PIN_RESET);//solenoid 1 on command
	HAL_GPIO_WritePin(GPIOE, LOW_SIDE_SW_1_Pin ,GPIO_PIN_RESET);//solenoid 2 on command
	}

if (((pkt.data[0] << 8) | pkt.data[1]) == 28) {

//	hx711_init(&LOAD_CELL1, LC1_CK_GPIO_Port, LC1_CK_Pin, LC1_DIN_GPIO_Port, LC1_DIN_Pin);
//	hx711_init(&LOAD_CELL2, LC2_CK_GPIO_Port, LC2_CK_Pin, LC2_DIN_GPIO_Port, LC2_DIN_Pin);
//	hx711_init(&LOAD_CELL3, LC3_CK_GPIO_Port, LC3_CK_Pin, LC3_DIN_GPIO_Port, LC3_DIN_Pin);
//	hx711_init(&LOAD_CELL4, LC4_CK_GPIO_Port, LC4_CK_Pin, LC4_DIN_GPIO_Port, LC4_DIN_Pin);
//
//
//	hx711_coef_set(&LOAD_CELL1, -45); // LC1
//	hx711_coef_set(&LOAD_CELL2, -44.5 ); // set calibration coefficent
//	hx711_coef_set(&LOAD_CELL3, -44.5); // set calibration coefficent
//	hx711_coef_set(&LOAD_CELL4, -44.5); // set calibration coefficent
//
//	hx711_tare(&LOAD_CELL1, 10); //read offset
//	hx711_tare(&LOAD_CELL2, 10); //read offset
//	hx711_tare(&LOAD_CELL3, 10); //read offset
//	hx711_tare(&LOAD_CELL4, 10); //read offset

	}
}
