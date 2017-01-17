// SBDBT/SBXBT Series PS3 / USB Controller Communication Program
//
// Copyright (C) 2014 Kunihiro Shibuya 
// Standalone Pairing modified (C) 2012 Toshihiro Nakatani
// added support controller (C) 2012 Toshihiro Nakatani
// added support controller (C) 2012 Michio Ono

#include <stdlib.h>
#define USE_AND_OR /* To enable AND_OR mask setting */
#include <ports.h>
#include <pps.h>
#include <outcompare.h>
#include "global.h"
#include "btstack/btdebug.h"
#include "btstack/btstack.h"
#include "btstack/hci_cmds.h"
#include "btstack/run_loop.h"
#include "btstack/sdp_util.h"
#include "btstack/hal_tick.h"

#include "btstack/hci.h"
#include "btstack/l2cap.h"
#include "btstack/btstack_memory.h"
#include "btstack/remote_device_db.h"
#include "btstack/rfcomm.h"
#include "btstack/sdp.h"
#include "btstack/hal_tick.h"

#include "TimeDelay.h"

#define	CREDITS					 10

#define	led1_on_mac()			{led1_on(); led1_on_count=clock(); led1_on_state=1;}
#if	USE_SPP_SERVER
static uint8_t		rfcomm_send_credit=CREDITS;
static uint8_t		rfcomm_channel_nr = 1;
static uint16_t		rfcomm_channel_id;
static uint8_t		__attribute__ ((aligned(2))) spp_service_buffer[128];
#endif

static volatile clock_t			timer_counter=0;
static unsigned					led1_on_count=0;
static uint8_t					led1_on_state=0;
char							lineBuffer[50*10];
static uint8_t					startup_state=0;
void (*usb_tick_handler)(void);	//sibu

#if	SEND_ON_DIFFERENT_DATA || NEUTRAL_DATA_SUPPRESS
static char					lastsendbuf[FORMAT_DATALEN];
#endif
#if	UART1_MINIMUM_INTERVAL
static clock_t					lastsendtime=0;
#endif

#define	BUTTON_STATE_INIT	0
WORD	sixaxis_control_channel_id=0;
static WORD	sixaxis_interrupt_channel_id=0;

hci_transport_t * hci_transport_picusb_instance();

#if defined(__PIC24FJ64GB004__)
	_CONFIG1(WDTPS_PS2048 & FWPSA_PR128 & WINDIS_OFF & FWDTENDEF & ICSDEF & GWRP_OFF & GCP_OFF & JTAGEN_OFF)
	_CONFIG2(POSCMOD_NONE & I2C1SEL_PRI & IOL1WAY_OFF & OSCIOFNC_ON & FCKSM_CSDCMD & FNOSC_FRCPLL & PLL96MHZ_ON & PLLDIV_DIV2 & IESO_OFF)
	_CONFIG3(WPFP_WPFP0 & SOSCSEL_IO & WUTSEL_LEG & WPDIS_WPDIS & WPCFG_WPCFGDIS & WPEND_WPENDMEM)
	_CONFIG4(DSWDTPS_DSWDTPS3 & DSWDTOSC_LPRC & RTCOSC_SOSC & DSBOREN_OFF & DSWDTEN_OFF)
#else

    #error Cannot define configuration bits.

#endif

BYTE        deviceAddress;  // Address of the device on the USB
bd_addr_t addr_global;

BOOL InitializeSystem ( void )
{
   #if defined(__PIC24FJ64GB004__)
	    {
	        unsigned int pll_startup_counter = 600;
	        CLKDIVbits.PLLEN = 1;
	        while(pll_startup_counter--);
	    }

		AD1PCFG = 0xffff;
	    CLKDIV = 0;    // Set PLL prescaler (1:1)
    #endif

	on_setup();
	led1_setup();
	led2_setup();
	led1_on();
	led2_on();
	u1rx_setup();
	u1tx_setup();
	u1cts_setup();
	u1rts_setup();
	sto_setup();
	assoc_setup();
	pairen_setup();

    // Init UART
    UART1Init(BAUDRATE1);
#if	STO_SERIAL
    UART2Init();
#endif

    return TRUE;
} // InitializeSystem

BOOL USB_ApplicationEventHandler ( BYTE address, USB_EVENT event, void *data, DWORD size )
{
    #ifdef USB_BLUETOOTH_SUPPORT_SERIAL_NUMBERS
        BYTE i;
    #endif

    // Handle specific events.
    switch ((int)event)
    {
        case EVENT_BLUETOOTH_ATTACH:
            if (size == sizeof(BLUETOOTH_DEVICE_ID))
            {
                deviceAddress   = ((BLUETOOTH_DEVICE_ID *)data)->deviceAddress;
                log_info( "Bluetooth device attached - event, deviceAddress=%d\r\n",deviceAddress );
                #ifdef USB_BLUETOOTH_SUPPORT_SERIAL_NUMBERS
                    for (i=1; i<((BLUETOOTH_DEVICE_ID *)data)->serialNumberLength; i++)
                    {
                        log_info("%c", ((BLUETOOTH_DEVICE_ID *)data)->serialNumber[i] );
                    }
	                log_info( "\r\n" );
                #endif
                return TRUE;
            }
            break;

        case EVENT_BLUETOOTH_DETACH:
            deviceAddress   = 0;
            log_info( "Bluetooth device detached - event\r\n" );
            return TRUE;

        case EVENT_BLUETOOTH_TX_CMD_DONE:           // The main state machine will poll the driver.
            return TRUE;

        case EVENT_BLUETOOTH_TX_ACL_DONE:           // The main state machine will poll the driver.
            return TRUE;

        case EVENT_BLUETOOTH_RX_EVENT_DONE:
			if(*(DWORD*)data){
				event_bluetooth_rxEvent_done((WORD)(*(DWORD*)data));
			}
            return TRUE;

        case EVENT_BLUETOOTH_RX_ACL_DONE:
			if(*(DWORD*)data){
				event_bluetooth_rxAcl_done((WORD)(*(DWORD*)data));
			}
            return TRUE;

        case EVENT_VBUS_REQUEST_POWER:
            // We'll let anything attach.
            return TRUE;

        case EVENT_VBUS_RELEASE_POWER:
            // We aren't keeping track of power.
            return TRUE;

        case EVENT_HUB_ATTACH:
            log_info( "\r\n***** USB Error - hubs are not supported *****\r\n" );
            return TRUE;
            break;

        case EVENT_UNSUPPORTED_DEVICE:
            log_info( "\r\n***** USB Error - device is not supported *****\r\n" );
            return TRUE;
            break;

        case EVENT_CANNOT_ENUMERATE:
            log_info( "\r\n***** USB Error - cannot enumerate device *****\r\n" );
            return TRUE;
            break;

        case EVENT_CLIENT_INIT_ERROR:
            log_info( "\r\n***** USB Error - client driver initialization error *****\r\n" );
            return TRUE;
            break;

        case EVENT_OUT_OF_MEMORY:
            log_info( "\r\n***** USB Error - out of heap memory *****\r\n" );
            return TRUE;
            break;

        case EVENT_UNSPECIFIED_ERROR:   // This should never be generated.
            log_info( "\r\n***** USB Error - unspecified *****\r\n" );
            return TRUE;
            break;

        case EVENT_SUSPEND:
        case EVENT_DETACH:
        case EVENT_RESUME:
        case EVENT_BUS_ERROR:
            return TRUE;
            break;

        default:
            break;
    }

    return FALSE;

} // USB_ApplicationEventHandler


#define	ANALOG_STATE_INIT		0x80

WORD sixaxis_button_state=BUTTON_STATE_INIT;
BYTE sixaxis_lx=ANALOG_STATE_INIT;
BYTE sixaxis_ly=ANALOG_STATE_INIT;
BYTE sixaxis_rx=ANALOG_STATE_INIT;
BYTE sixaxis_ry=ANALOG_STATE_INIT;

uint8_t getsum(void *buf,size_t size)
{
	const uint8_t	*p=(uint8_t*)buf;
	uint8_t			ret=0;

	for(;size;size--){
		ret+=*p++;
	}
	return ret;
}

uint8_t conv_analog(uint8_t ai)
{
	uint8_t ret;

#if	STICK_NUTRAL_VALUE==64
	ai>>=1;
#endif
	if(ai>(STICK_NUTRAL_VALUE+STICK_DEADZONE)) {
	#if	USE_STICK_FOR_DIGITAL
		//スティックをデジタル的に扱う
		ret=STICK_MAXIMUM_VALUE;
	#else
		//スティックをアナログ的に扱う
		#if STICK_RESOLUTION
			ret=((ai+(STICK_RESOLUTION-1))/STICK_RESOLUTION)*STICK_RESOLUTION-1;
		#else
			ret=ai;
		#endif
		#if	STICK_MAXIMUM_VALUE<255
		if(ret>STICK_MAXIMUM_VALUE){
			ret=STICK_MAXIMUM_VALUE;
		}
		#endif
	#endif
	} else if(ai<(STICK_NUTRAL_VALUE-STICK_DEADZONE)) {
	#if	USE_STICK_FOR_DIGITAL
		//スティックをデジタル的に扱う
		ret=STICK_MINIMUM_VALUE;
	#else
		//スティックをアナログ的に扱う
		#if STICK_RESOLUTION
			ret=((ai/STICK_RESOLUTION)*STICK_RESOLUTION;
		#else
			ret=ai;
		#endif
		#if	STICK_MINIMUM_VALUE
			if(ret<STICK_MINIMUM_VALUE){
				ret=STICK_MINIMUM_VALUE;
			}
		#endif
	#endif
	} else {
		ret=STICK_NUTRAL_VALUE;	//ニュートラルとして扱う
	}

	return ret;
}

//ニュートラル時のデータ
#if FORMAT_RCB3
	const uint8_t neutral_packet[]={0x80,0x00,0x00,0x40,0x40,0x40,0x40,};
#elif	FORMAT_RCB4
	const uint8_t neutral_packet[]={0x0D,0x00,0x02,0x50,0x03,0x00,0x00,0x00,0x40,0x40,0x40,0x40,};	
#elif	FORMAT_RPU
	const uint8_t neutral_packet[]={0x4B,0xFF,0xFF,0xFF,0x80,0x80,0x80,0x80,0xF3};	
	const uint8_t disconnect_packet[]={0x4B,0xFF,0xFF,0xFF,0x80,0x80,0x80,0x80,0xFF};	
	const uint8_t connect_packet[]={0x4B,0xFF,0xFF,0xFF,0x80,0x80,0x80,0x80,0xFB};	
#endif

clock_t clock(void)
{
	return timer_counter;
}

void UART1PutBuf(char *buf,size_t size)
{
#if	UART1_MINIMUM_INTERVAL
	for(;(clock()-lastsendtime)<UART1_MINIMUM_INTERVAL;){
	}
	lastsendtime=clock();
#endif

	for(;size;size--){
		UART1PutChar(*buf++);
	}
	return;
}

void out_button_state(void)
{
	int	pos=0;
	//電文　固定値
#if FORMAT_RCB3
	lineBuffer[pos++]=0x80; //1バイト目:0x80固定
#elif	FORMAT_RCB4
	lineBuffer[pos++]=0x0D;
	lineBuffer[pos++]=0x00;
	lineBuffer[pos++]=0x02;
	lineBuffer[pos++]=0x50;
	lineBuffer[pos++]=0x03;
	lineBuffer[pos++]=0x00;
#elif	FORMAT_RPU
	lineBuffer[pos++]=0x4B;
	lineBuffer[pos++]=0xFF;
#else
#error	"Unknown format"
#endif
	//電文の作成
#if FORMAT_RCB3
	lineBuffer[pos++]=(sixaxis_button_state>>8)&0xff;	//2バイト目:ボタン状態デジタル1
	lineBuffer[pos++]=(sixaxis_button_state>>0)&0xff;	//3バイト目:ボタン状態デジタル2
	//xyxy
	lineBuffer[pos++]=conv_analog(sixaxis_lx);			//4バイト目:左スティックX状態(0x00-0x7f) アナログのようでデジタル
	lineBuffer[pos++]=conv_analog(sixaxis_ly);			//5バイト目:左スティックY状態(0x00-0x7f) アナログのようでデジタル
	lineBuffer[pos++]=conv_analog(sixaxis_rx);			//6バイト目:右スティックX状態(0x00-0x7f) アナログのようでデジタル
	lineBuffer[pos++]=conv_analog(sixaxis_ry);			//7バイト目:右スティックY状態(0x00-0x7f) アナログのようでデジタル
	lineBuffer[pos]=(getsum(lineBuffer,pos)&0x7f);		//8バイト目:チェックサム下位7ビット
	pos++;
#elif	FORMAT_RCB4
	lineBuffer[pos++]=(sixaxis_button_state>>8)&0xff;	//2バイト目:ボタン状態デジタル1
	lineBuffer[pos++]=(sixaxis_button_state>>0)&0xff;	//3バイト目:ボタン状態デジタル2
	//yxyx
	lineBuffer[pos++]=conv_analog(0xFF-sixaxis_ly);		//4バイト目:左スティックy状態(0x00-0x7f) アナログのようでデジタル
	lineBuffer[pos++]=conv_analog(sixaxis_lx);			//5バイト目:左スティックx状態(0x00-0x7f) アナログのようでデジタル
	lineBuffer[pos++]=conv_analog(0xFF-sixaxis_ry);		//6バイト目:右スティックy状態(0x00-0x7f) アナログのようでデジタル
	lineBuffer[pos++]=conv_analog(sixaxis_rx);			//7バイト目:右スティックx状態(0x00-0x7f) アナログのようでデジタル
	lineBuffer[pos]=(getsum(lineBuffer,pos)&0xFF);		//8バイト目:チェックサム
	pos++;
#elif	FORMAT_RPU
	lineBuffer[pos++]=((sixaxis_button_state>>8)&0xff)^0xff;	//2バイト目:ボタン状態デジタル1
	lineBuffer[pos++]=((sixaxis_button_state>>0)&0xff)^0xff;	//3バイト目:ボタン状態デジタル2
	lineBuffer[pos++]=conv_analog(sixaxis_rx);			//5バイト目:右スティックX状態(0x00-0x7f) アナログのようでデジタル
	lineBuffer[pos++]=conv_analog(sixaxis_ry);			//6バイト目:右スティックY状態(0x00-0x7f) アナログのようでデジタル
	lineBuffer[pos++]=conv_analog(sixaxis_lx);			//7バイト目:左スティックX状態(0x00-0x7f) アナログのようでデジタル
	lineBuffer[pos++]=conv_analog(sixaxis_ly);			//8バイト目:左スティックY状態(0x00-0x7f) アナログのようでデジタル
	lineBuffer[pos++]=0xF3;								//9バイト目:0xF3固定
#endif

#if	SEND_ON_DIFFERENT_DATA
	//前回と差があった時のみ送信
	if(memcmp(lastsendbuf,lineBuffer,pos)){			//前回送信データと差がある場合のみ送信
		led1_on_mac();									//赤色LED点灯
		UART1PutBuf(lineBuffer,pos);						//データ送信

#if	NEUTRAL_ADDITIONAL_PACKETS
		if(!memcmp(neutral_packet,lineBuffer,sizeof(neutral_packet))){	//このデータはニュートラルか
			unsigned nc;
			for(nc=0;nc<NEUTRAL_ADDITIONAL_PACKETS;nc++){
				UART1PutBuf(lineBuffer,pos);				//ニュートラルのデータの場合は追加で送る
			}
		}
#endif
		memcpy(lastsendbuf,lineBuffer,pos);				//送信したデータを比較用に保存
	}
#else
	//連続送信
#if	UART1_MINIMUM_INTERVAL
	if((clock()-lastsendtime)>=UART1_MINIMUM_INTERVAL){
#endif
#if	NEUTRAL_DATA_SUPPRESS
		if(memcmp(neutral_packet,lineBuffer,sizeof(neutral_packet)) || memcmp(lastsendbuf,lineBuffer,pos)){	//このデータはニュートラルか
#endif
			if(UART1GetBufferSpace()>=pos){
				led1_on_mac();									//赤色LED点灯
				UART1PutBuf(lineBuffer,pos);						//データ送信

#if	NEUTRAL_ADDITIONAL_PACKETS
				if(!memcmp(neutral_packet,lineBuffer,sizeof(neutral_packet))){	//このデータはニュートラルか
					unsigned nc;
					for(nc=0;nc<NEUTRAL_ADDITIONAL_PACKETS;nc++){
						UART1PutBuf(lineBuffer,pos);				//ニュートラルのデータの場合は追加で送る
					}
				}
#endif
#if	NEUTRAL_DATA_SUPPRESS
				memcpy(lastsendbuf,lineBuffer,pos);				//送信したデータを比較用に保存
#endif
			}
#if	NEUTRAL_DATA_SUPPRESS
		}
#endif
#if	UART1_MINIMUM_INTERVAL
	}
#endif
#endif

	return;
}

void init_button_state(void)
{
	sixaxis_button_state=BUTTON_STATE_INIT;
	sixaxis_lx=ANALOG_STATE_INIT;
	sixaxis_ly=ANALOG_STATE_INIT;
	sixaxis_rx=ANALOG_STATE_INIT;
	sixaxis_ry=ANALOG_STATE_INIT;

#if	FORMAT_RPU
	memcpy(lineBuffer,disconnect_packet,sizeof(disconnect_packet));
#if	SEND_ON_DIFFERENT_DATA || NEUTRAL_DATA_SUPPRESS
	memcpy(lastsendbuf,disconnect_packet,sizeof(disconnect_packet));
#endif
	UART1PutBuf(lineBuffer,sizeof(disconnect_packet));
#else
	out_button_state();
#endif
}

static int enable_sixaxis(uint16_t channel)
{
    uint8_t cmd_buf[6];
    cmd_buf[0] = 0x53;// HID BT Set_report (0x50) | Report Type (Feature 0x03)
    cmd_buf[1] = 0xF4;// Report ID
    cmd_buf[2] = 0x42;// Special PS3 Controller enable commands
    cmd_buf[3] = 0x03;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;

	return l2cap_send_internal(channel,cmd_buf,6);
}

static int set_sixaxis_led(uint16_t channel,int led)
{
	memcpy(lineBuffer+2,OUTPUT_REPORT_BUFFER,OUTPUT_REPORT_BUFFER_SIZE);
    lineBuffer[0] = 0x52;// HID BT Set_report (0x50) | Report Type (Output 0x02)
    lineBuffer[1] = 0x01;// Report ID
	lineBuffer[11] |= (uint8_t)(((uint16_t)led & 0x0f) << 1);    

	return l2cap_send_internal(channel,(uint8_t*)lineBuffer,HID_BUFFERSIZE);
}

static uint8_t sixaxis_init_state=0;

static void l2cap_control_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
	if (packet_type == HCI_EVENT_PACKET){
		switch(packet[0]){
		case L2CAP_EVENT_INCOMING_CONNECTION:
			log_info("L2CAP_EVENT_INCOMING_CONNECTION\n");
			l2cap_accept_connection_internal(channel);
			break;
		case L2CAP_EVENT_CHANNEL_OPENED:
	        if (packet[2]) {
	            log_info("control Connection failed\n\r");
	            return;
		    }
			log_info("control Connected\n\r");
			sixaxis_control_channel_id=channel;
			break;
		case L2CAP_EVENT_CHANNEL_CLOSED:
			log_info("L2CAP_CHANNEL_CLOSED\n");
			sixaxis_control_channel_id=0;
			sixaxis_init_state=0;
			init_button_state();
			break;
		case L2CAP_EVENT_CREDITS:
			switch(sixaxis_init_state){
			case	1:
				sixaxis_init_state++;
				DelayMs(1000);
				set_sixaxis_led(channel,1);
#if	FORMAT_RPU
				memcpy(lineBuffer,connect_packet,sizeof(connect_packet));
#if	SEND_ON_DIFFERENT_DATA || NEUTRAL_DATA_SUPPRESS
				memcpy(lastsendbuf,connect_packet,sizeof(connect_packet));
#endif
				UART1PutBuf(lineBuffer,sizeof(connect_packet));
#endif
				break;
			default:
				break;
			}
			break;
		default:
			log_info("l2cap:unknown(%02x)\n",packet[0]);
			break;
		}	
	}
}

static void l2cap_interrupt_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
	if (packet_type == HCI_EVENT_PACKET){
		switch(packet[0]){
		case L2CAP_EVENT_INCOMING_CONNECTION:
			log_info("L2CAP_EVENT_INCOMING_CONNECTION\n");
			l2cap_accept_connection_internal(channel);
			break;
		case L2CAP_EVENT_CHANNEL_OPENED:
	        if (packet[2]) {
	            log_info("interrupt Connection failed\n\r");
	            return;
		    }
			log_info("interrupt Connected\n\r");
			sixaxis_interrupt_channel_id=channel;
			break;
		case L2CAP_EVENT_CHANNEL_CLOSED:
			log_info("L2CAP_CHANNEL_CLOSED\n");
			sixaxis_interrupt_channel_id=0;
			sixaxis_init_state=0;
			init_button_state();
			break;
		case L2CAP_EVENT_CREDITS:
			switch(sixaxis_init_state){
			case	0:
				sixaxis_init_state++;
				enable_sixaxis(sixaxis_control_channel_id);
				break;
			default:
				break;
			}			break;
		default:
			log_info("l2cap:unknown(%02x)\n",packet[0]);
			break;
		}	
	}
	if (packet_type == L2CAP_DATA_PACKET && size && packet[0]==0xa1){
		sixaxis_process_packet(packet+1,size-1);
	}
}

// Bluetooth logic
static void packet_handler (void * connection, uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
	bd_addr_t	event_addr;
#if	USE_SPP_SERVER
    uint8_t		rfcomm_channel_nr;
    uint8_t		rfcomm_channel_id_temp;
    uint16_t	mtu;
#endif

	switch (packet_type) {
		case HCI_EVENT_PACKET:
			switch (packet[0]) {
				case BTSTACK_EVENT_STATE:
					// bt stack activated, get started - set local name
					if (packet[2] == HCI_STATE_WORKING) {
#ifndef	__DEBUG
						if(
							!addr_global[0]
						&&	!addr_global[1]
						&&	!addr_global[2]
						&&	!addr_global[3]
						&&	!addr_global[4]
						&&	!addr_global[5]
						){
							//bd address failed -> reset
							Reset();
						}
#endif
						xsprintf(lineBuffer,LOCAL_NAME "-%02x%02x%02x%02x%02x%02x",
							addr_global[0],
							addr_global[1],
							addr_global[2],
							addr_global[3],
							addr_global[4],
							addr_global[5]
						);
						#if STANDALONE_PAIRING
						save_local_bluetooth_address(addr_global);
						#endif

						log_info("local name:%s\r\n",lineBuffer);
#ifndef	__DEBUG
						DelayMs(100);
#endif
                        hci_send_cmd(&hci_write_local_name, lineBuffer);
					}
					break;
				
				case HCI_EVENT_COMMAND_COMPLETE:
					if (COMMAND_COMPLETE_EVENT(packet, hci_read_bd_addr)){
                        bt_flip_addr(event_addr, &packet[6]);
                        log_info("BD-ADDR: %s\n\r", bd_addr_to_str(event_addr));
#if !USE_SPP_SERVER
        				startup_state=1;
#endif
                        break;
                    }
#if	USE_SPP_SERVER
					if (COMMAND_COMPLETE_EVENT(packet, hci_write_local_name)){
    				    hci_send_cmd(&hci_write_class_of_device, 0);
                        break;
                    }
					if (COMMAND_COMPLETE_EVENT(packet, hci_write_class_of_device)){
						if(get_pairen()){
							hci_discoverable_control(1);
						}
						startup_state=1;
//						hci_send_cmd(&hci_write_authentication_enable, 1);
                        break;
					}
//					if (COMMAND_COMPLETE_EVENT(packet, hci_write_authentication_enable)){
//						hci_discoverable_control(1);
//						break;
//					}
#endif
                    break;
				case HCI_EVENT_LINK_KEY_REQUEST:
					// deny link key request
                    log_info("Link key request\n\r");
                    bt_flip_addr(event_addr, &packet[2]);
					hci_send_cmd(&hci_link_key_request_negative_reply, &event_addr);
					break;
					
				case HCI_EVENT_PIN_CODE_REQUEST:
					// inform about pin code request
                    bt_flip_addr(event_addr, &packet[2]);
                    log_info("Pin code request - using '" PIN_CODE_DEFAULT "'\n\r");
					hci_send_cmd(&hci_pin_code_request_reply, &event_addr, 4, PIN_CODE_DEFAULT);
					break;
#if	USE_SPP_SERVER
                case RFCOMM_EVENT_INCOMING_CONNECTION:
					// data: event (8), len(8), address(48), channel (8), rfcomm_cid (16)
					bt_flip_addr(event_addr, &packet[2]); 
					rfcomm_channel_nr = packet[8];
					rfcomm_channel_id_temp = READ_BT_16(packet, 9);
					log_info("RFCOMM channel %u requested for %s\n\r", rfcomm_channel_nr, bd_addr_to_str(event_addr));
                    rfcomm_accept_connection_internal(rfcomm_channel_id_temp);
					break;
					
				case RFCOMM_EVENT_OPEN_CHANNEL_COMPLETE:
					// data: event(8), len(8), status (8), address (48), server channel(8), rfcomm_cid(16), max frame size(16)
					if (packet[2]) {
						log_info("RFCOMM channel open failed, status %u\n\r", packet[2]);
					} else {
						rfcomm_channel_id = READ_BT_16(packet, 12);
						mtu = READ_BT_16(packet, 14);
						log_info("\n\rRFCOMM channel open succeeded. New RFCOMM Channel ID %u, max frame size %u\n\r", rfcomm_channel_id, mtu);
					}
					break;
                    
                case RFCOMM_EVENT_CHANNEL_CLOSED:
                    rfcomm_channel_id = 0;
                    break;
#endif
                default:
                    break;
			}
            break;
#if	USE_SPP_SERVER
        case RFCOMM_DATA_PACKET:
            // hack: truncate data (we know that the packet is at least on byte bigger
            //packet[size] = 0;
            //log_info( (const char *) packet);
		    led1_on_mac();
			for(;size--;){
				UART1PutChar(*packet++);
			}
			if(!--rfcomm_send_credit){
	            rfcomm_send_credit = CREDITS;
		        rfcomm_grant_credits(rfcomm_channel_id, rfcomm_send_credit);
			}
			break;
#endif
        default:
            break;
	}
}

int main ( void )
{
	RestartWatchdog();

#ifdef __DEBUG
	xdev_out(UART2PutChar);
#endif

    // Initialize the processor and peripherals.
    if ( InitializeSystem() != TRUE )
    {
        log_info( "\r\n\r\nCould not initialize " LOCAL_NAME " - system.  Halting.\r\n\r\n" );
#ifdef	__DEBUG
        while (1);
#else
		Reset();
#endif
	}
    if ( USBHostInit(0) == TRUE )
    {
        log_info( "\r\n\r\n***** " LOCAL_NAME " Initialized *****\r\n\r\n" );
    }
    else
    {
        log_info( "\r\n\r\nCould not initialize " LOCAL_NAME " - USB.  Halting.\r\n\r\n" );
#ifdef	__DEBUG
        while (1);
#else
		Reset();
#endif
    }

   // Main Processing Loop
    while(1)
    {
		if(deviceAddress){
#if	USE_USB_HID_CONTROLLER
			if(gc_DevData.ID.vid == 0x05B8 && gc_DevData.ID.pid == 0x1004){//add @naka_at_kure
				dev_elecom_u3312s();
			}else if(gc_DevData.ID.vid == 0x05B8 && gc_DevData.ID.pid == 0x1006){//add @naka_at_kure
				dev_elecom_u3412s();
			}else if(gc_DevData.ID.vid == 0x0079 && gc_DevData.ID.pid == 0x0006){//add @micutil
				dev_dragonrise();
			}else if(gc_DevData.ID.vid == 0x046D){
				if(gc_DevData.ID.pid == 0xC219){//add @micutil
					dev_logicool_f710();
				}else if(gc_DevData.ID.pid == 0xC218){//add @micutil
					dev_logicool_f510();
				}else if(gc_DevData.ID.pid == 0xC21F){// logicool f710 X mode
				}else if(gc_DevData.ID.pid == 0xC21E){// logicool f510 X mode
				}else if(gc_DevData.ID.pid == 0xC22F){// logicool f710 and f510 initialize
				}else{
					log_info( "\r\n usb controller not found\r\n\r\n" );
					break;
				}
			}else
#endif
			//connect PS3Controller
			if(gc_DevData.ID.vid == 0x054Cul && gc_DevData.ID.pid == 0x0268ul){
				dev_sony_ps3();
			}else{
				log_info( "\r\n usb controller not found\r\n\r\n" );
				break;
			}
		}
        // This demo does not check for overcurrent conditions.  See the
        // USB Host - Data Logger for an example of overcurrent detection
        // with the PIC24F and the USB PICtail Plus.

        // Maintain USB Host State
        USBHostTasks();
		//Idle();
        //if (CheckForNewAttach()){
		//	break;
        //}
    }

	init_button_state();
	DelayMs(1000);

	//led2_off();

	/// GET STARTED with BTstack ///
	btstack_memory_init();
    run_loop_init(RUN_LOOP_EMBEDDED);
	
    // init HCI
	hci_transport_t    * transport = hci_transport_picusb_instance();
	bt_control_t       * control   = NULL;
    hci_uart_config_t  * config    = NULL;
    remote_device_db_t * remote_db = (remote_device_db_t *) &remote_device_db_memory;
	hci_init(transport, config, control, remote_db);

    // use eHCILL
//    bt_control_cc256x_enable_ehcill(1);
    
    // init L2CAP
    l2cap_init();
    l2cap_register_packet_handler(packet_handler);

	l2cap_register_service_internal(NULL, l2cap_control_packet_handler, PSM_HID_CONTROL, 160);
	l2cap_register_service_internal(NULL, l2cap_interrupt_packet_handler, PSM_HID_INTERRUPT, 160);

#if	USE_SPP_SERVER
	// init RFCOMM
    rfcomm_init();
    rfcomm_register_packet_handler(packet_handler);
    rfcomm_register_service_with_initial_credits_internal(NULL, rfcomm_channel_nr, 100, CREDITS);  // reserved channel, mtu=100, 1 credit

    // init SDP, create record for SPP and register with SDP
    sdp_init();

	memset(spp_service_buffer, 0, sizeof(spp_service_buffer));
    service_record_item_t * service_record_item = (service_record_item_t *) spp_service_buffer;
    sdp_create_spp_service( (uint8_t*) &service_record_item->service_record, 1, "SPP");
    log_info("SDP service buffer size: %u\n\r", (uint16_t) (sizeof(service_record_item_t) + de_get_len((uint8_t*) &service_record_item->service_record)));
    sdp_register_service_internal(NULL, service_record_item);
#endif

	// turn on!
	hci_power_control(HCI_POWER_ON);

#if	!USE_WRITE_STORED_LINK_KEY
extern linked_list_t db_mem_link_keys;
void load_link_keys(remote_device_db_t * lpremote_device_db);
void print_link_keys(linked_list_t list);
	load_link_keys(remote_db);
	print_link_keys(db_mem_link_keys);
#endif

    led1_off();

//DelayMs(1000);

    // go!
    run_loop_execute();	

    return 0;

} // main


void hal_tick_init(void)
{
	usb_tick_handler=NULL;
}

void hal_tick_set_handler(void (*tick_handler)(void))
{
	usb_tick_handler=tick_handler;
}

void hal_tick_event(void)
{
	timer_counter++;

	RestartWatchdog();

#ifndef	__DEBUG
	if(!startup_state && timer_counter>5000){
		Reset();
	}
#endif

    if (sixaxis_control_channel_id){
        if(timer_counter&0x100){
            led2_off();
			assoc_l();
        }else{
            led2_on();
			assoc_h();
        }
		sto_h();
    }else{
        led2_on();
		sto_l();
		assoc_l();
    }

	if(led1_on_state && (timer_counter-led1_on_count)>LED1_ON_COUNT){
		led1_off();
		led1_on_state=0;
	}

	// call tick handler
	if(usb_tick_handler){
		usb_tick_handler();
	}
}

#if	USE_SPP_SERVER
void sendchar(void)
{
	int	ch;
	unsigned len;

	if(is_rfcomm_send_avail(rfcomm_channel_id)){
		//can't send rfcomm now
		return;
	}

	for(len=0;len<sizeof(lineBuffer);len++){
		ch=UART1GetChar();
		if(ch<0){
			break;
		}
		//log_info("UART1GetChar=%c\r\n",ch);
		lineBuffer[len]=ch;
	}
	if(len){
	    led1_on_mac();
        int err = rfcomm_send_internal(rfcomm_channel_id, (uint8_t*) lineBuffer, len);
        if (err) {
            log_error("rfcomm_send_internal -> error %d", err);
        }
	}
}
#endif
/*************************************************************************
 * EOF main.c
 */

