#ifndef __NET_H
#define __NET_H
//--------------------------------------------------
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "enc28j60.h"
#include "stdio.h"

#define IP_ADDR {192,168,1,197}

typedef struct enc28j60_frame{
	uint8_t addr_dest[6];
	uint8_t addr_src[6];
	uint16_t type;
	uint8_t data[];
} 	enc28j60_frame_ptr;

typedef struct arp_msg{
  uint16_t net_tp;
  uint16_t proto_tp;
  uint8_t macaddr_len;
  uint8_t ipaddr_len;
  uint16_t op;
  uint8_t macaddr_src[6];
  uint8_t ipaddr_src[4];
  uint8_t macaddr_dst[6];
  uint8_t ipaddr_dst[4];
} arp_msg_ptr;

typedef struct ip_pkt{

uint8_t verlen;		// protokol versiyonu ve başlık uzunluğu
uint8_t ts;			// sevris türü
uint16_t len; 		// uzunluk
uint16_t id; 		// paket kimliği
uint16_t fl_frg_of; // bayraklar ve parça ofseti
uint8_t ttl; 		// ömür boyu
uint8_t prt; 		// protokol tipi
uint16_t cs;		 // başlık sağlama toplamı
uint8_t ipaddr_src[4]; // Gönderenin IP adresi
uint8_t ipaddr_dst[4]; // Hedef IP adresi
uint8_t data[]; 	// veri
} ip_pkt_ptr;

typedef struct icmp_pkt{

uint8_t msg_tp; 	// servis türü
uint8_t msg_cd;		// mesaj kodu
uint16_t cs; 		// başlık sağlama toplamı
uint16_t id; 		// paket kimliği
uint16_t num; 		// paket numarası
uint8_t data[]; 	// veri
} icmp_pkt_ptr;

#define be16toword(a) ((((a)>>8)&0xff)|(((a)<<8)&0xff00))

#define ETH_ARP be16toword(0x0806)
#define ETH_IP be16toword(0x0800)

#define ARP_ETH be16toword(0x0001)
#define ARP_IP be16toword(0x0800)
#define ARP_REQUEST be16toword(1)
#define ARP_REPLY be16toword(2)

#define IP_ICMP 1
#define IP_TCP 6
#define IP_UDP 17

#define ICMP_REQ 8
#define ICMP_REPLY 0

void net_ini(void);
void net_pool(void);
//--------------------------------------------------
#endif /* __NET_H */
