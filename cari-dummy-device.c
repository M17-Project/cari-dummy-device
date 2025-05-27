/*
 * cari-dummy-device.c
 *
 *  Edited on: May 26, 2025
 *  Author: Wojciech Kaczmarski, SP5WWP
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <zmq.h>

#include "term.h" //colored terminal font

#define DBG_HALT            while(1);
#define CARI_VER            (((uint8_t)1<<4) | 3)   //CARI v1.3
#define CARI_DEV_IDENT      "CARI dummy device, Woj SP5WWP"

typedef enum
{
    CMD_PING,
    CMD_DEV_SET_REG,
    CMD_SUB_SET_PARAM,
    CMD_SUB_EXEC,
    CMD_SUB_CONN,
    CMD_SUB_START_BB_STREAM,
    CMD_DEV_START_SPVN_STREAM,

    CMD_DEV_GET_IDENT = 0x80,
    CMD_DEV_GET_REG,
    CMD_SUB_GET_CAPS,
    CMD_SUB_GET_PARAM,
    CMD_DEV_GET_SPVN_LIST
} cid_t;

typedef enum
{
    CARI_OK,
    CARI_MALFORMED,
    CARI_UNSUPPORTED,
    CARI_BIND_FAIL,
    CARI_CONNECTION_FAIL,
    CARI_OUT_OF_RANGE
} cari_err_t;

#define DEV_PLL_LOCK_ERR        (1<<0)
#define DEV_SUBDEV_COMM_ERR     (1<<1)
#define DEV_OVERHEAT_ERR        (1<<2)
#define DEV_FREQ_REF_ERR        (1<<3)

typedef enum
{
    DEV_COMPRESSION,
    DEV_SUPERVISION
} dev_cap_t;

typedef enum
{
    SUB_RESET,

    SUB_IQ_MOD,
    SUB_RX,
    SUB_TX,
    SUB_DUPLEX,
    SUB_AGC,
    SUB_AFC,
    SUB_FREQ_REF,
        
    SUB_AM_DEMOD,
    SUB_FM_DEMOD,
    SUB_PM_DEMOD,
    SUB_SSB_DEMOD,
        
    SUB_AM_MOD,
    SUB_FM_MOD,
    SUB_PM_MOD,
    SUB_SSB_MOD
} subdev_cap_t;

typedef enum
{
    SUB_ACT_RESET,
    SUB_ACT_RX_START,
    SUB_ACT_RX_STOP
} sub_act_t;

typedef struct
{
    uint16_t ul_port, dl_port, ctrl_port, spvn_port;
    int zmq_byte_cnt;
    void *zmq_ctx, *zmq_ul, *zmq_dl, *zmq_ctrl, *zmq_spvn;
    uint8_t ul_ok, dl_ok, ctrl_ok, spvn_ok;
    uint8_t zmq_buff[1024];
} cari_t;

//default settings
cari_t cari =
{
    .ul_port = 17000,
    .dl_port = 17001,
    .ctrl_port = 17002,
    .spvn_port = 17003
};

typedef struct
{
    uint64_t        rx_frequency;           //Hz
    uint64_t        tx_frequency;           //Hz
    float           lna_gain;               //dB
    float           power;                  //dBm
    float           ch_width;               //Hz
    float           samp_rate;              //Hz
    float           f_corr;                 //ppm
    uint8_t         num_capabilities;       //number of capabilities
    subdev_cap_t    capabilities[16];       //capabilities
} subdevice_t;

typedef struct
{
    char ident[128];
    uint8_t cari_version;
    uint8_t num_capabilities;
    dev_cap_t capabilities[16];
    uint8_t subdevices;
    subdevice_t subdevice[];
} device_t;

//sample settings
device_t device =
{
    CARI_DEV_IDENT,
    CARI_VER,               //CARI version
    1,                      //number of capabilities
    {DEV_SUPERVISION},      //supervision channel available
    2, {
    {0.0f, 438.8125e6, 0.0f, 30.0f, 12.5e3, 125e3, 0.0f, 5, {SUB_TX, SUB_FM_MOD, SUB_FM_DEMOD, SUB_AFC, SUB_AGC}},
    {431.2125e6, 0.0f, 0.0f, 30.0f, 12.5e3, 125e3, 0.0f, 5, {SUB_RX, SUB_FM_MOD, SUB_FM_DEMOD, SUB_AFC, SUB_AGC}}}
}; 

//debug printf
void dbg_print(const char* color_code, const char* fmt, ...)
{
	char str[200];
	va_list ap;

	va_start(ap, fmt);
	vsprintf(str, fmt, ap);
	va_end(ap);

	if(color_code!=NULL)
	{
		fputs(color_code, stdout);
		fputs(str, stdout);
		fputs(TERM_DEFAULT, stdout);
	}
	else
	{
		fputs(str, stdout);
	}
}

void cari_init(cari_t *cari)
{
    char addr[128];
    int res;

    cari->zmq_ctx = zmq_ctx_new();
    cari->zmq_ul = zmq_socket(cari->zmq_ctx, ZMQ_SUB);
    cari->zmq_dl = zmq_socket(cari->zmq_ctx, ZMQ_PUB);
    cari->zmq_ctrl = zmq_socket(cari->zmq_ctx, ZMQ_REP);
	cari->zmq_spvn = zmq_socket(cari->zmq_ctx, ZMQ_PUB);

    //downlink plane init
    sprintf(addr, "tcp://*:%d", cari->dl_port);
    res = zmq_bind(cari->zmq_dl, addr);

    if(res)
    {
        cari->dl_ok = 0;
    }
    else
    {
        cari->dl_ok = 1;
    }

    //control plane init
    sprintf(addr, "tcp://*:%d", cari->ctrl_port);
    res = zmq_bind(cari->zmq_ctrl, addr);

    if(res)
    {
        cari->ctrl_ok = 0;
    }
    else
    {
        cari->ctrl_ok = 1;
    }

    //supervision plane init
    sprintf(addr, "tcp://*:%d", cari->spvn_port);
    res = zmq_bind(cari->zmq_spvn, addr);

    if(res)
    {
        cari->spvn_ok = 0;
    }
    else
    {
        cari->spvn_ok = 1;
    }   
}

void dispSettings(device_t *dev, cari_t *cari)
{
    dbg_print(TERM_CLR, "");

    dbg_print(TERM_BLUE, "**CARI dummy device**\n");

    dbg_print(TERM_DEFAULT, "Ident: ");
    dbg_print(TERM_YELLOW, "%s\n", dev->ident);

    dbg_print(TERM_DEFAULT, "CARI version: ");
    dbg_print(TERM_YELLOW, "%d.%d\n", (dev->cari_version)>>4, (dev->cari_version)&0xF);

    dbg_print(TERM_DEFAULT, "Capabilities: ");
    for(uint8_t i=0; i<dev->num_capabilities; i++)
        dbg_print(TERM_YELLOW, "%02X " , dev->capabilities[i]);
    dbg_print(TERM_DEFAULT, "\n");

    dbg_print(TERM_DEFAULT, "Subdevices: ");
    dbg_print(TERM_YELLOW, "%d " , dev->subdevices);
    dbg_print(TERM_DEFAULT, "\n");

    //zmq planes
    dbg_print(TERM_DEFAULT, "DL   port: ");
    dbg_print(TERM_YELLOW, "%d", cari->dl_port);
    dbg_print(TERM_DEFAULT, ", status ");
    if(cari->dl_ok)
        dbg_print(TERM_GREEN, "OK\n");
    else
        dbg_print(TERM_YELLOW, "ERROR\n");

    dbg_print(TERM_DEFAULT, "CTRL port: ");
    dbg_print(TERM_YELLOW, "%d", cari->ctrl_port);
    dbg_print(TERM_DEFAULT, ", status ");
    if(cari->ctrl_ok)
        dbg_print(TERM_GREEN, "OK\n");
    else
        dbg_print(TERM_YELLOW, "ERROR\n");

    dbg_print(TERM_DEFAULT, "SPVN port: ");
    dbg_print(TERM_YELLOW, "%d", cari->spvn_port);
    dbg_print(TERM_DEFAULT, ", status ");
    if(cari->spvn_ok)
        dbg_print(TERM_GREEN, "OK\n");
    else
        dbg_print(TERM_YELLOW, "ERROR\n");
    printf("\n");

    //device info
    uint8_t num_devices = dev->subdevices;

    for(uint8_t i=0; i<num_devices; i++)
    {
        dbg_print(TERM_DEFAULT, "----------------------------------------");
    }
    printf("\n");

    for(uint8_t i=0; i<num_devices; i++)
    {
        dbg_print(TERM_DEFAULT, "| Subdevice ");
        dbg_print(TERM_YELLOW, "%d", i);
        dbg_print(TERM_DEFAULT, "                          |");

    }
    printf("\n");

    for(uint8_t i=0; i<num_devices; i++)
    {
        dbg_print(TERM_DEFAULT, "----------------------------------------");
    }
    printf("\n");

    for(uint8_t i=0; i<num_devices; i++)
    {
        char line[100] = {0};
        sprintf(line, "| Capabilities: ");
        for(uint8_t j=0; j<dev->subdevice[i].num_capabilities; j++)
            sprintf(&line[strlen(line)], "%02X ", dev->subdevice[i].capabilities[j]);
        for(uint8_t j=strlen(line); j<39; j++)
            line[j]=' ';
        sprintf(&line[strlen(line)], "|");

        dbg_print(TERM_DEFAULT, "%s", line);
    }
    printf("\n");

    for(uint8_t i=0; i<num_devices; i++)
    {
        char line[100] = {0};
        sprintf(line, "| RX frequency: %lu Hz", dev->subdevice[i].rx_frequency);
        for(uint8_t j=strlen(line); j<39; j++)
            line[j]=' ';
        sprintf(&line[strlen(line)], "|");

        dbg_print(TERM_DEFAULT, "%s", line);
    }
    printf("\n");

    for(uint8_t i=0; i<num_devices; i++)
    {
        char line[100] = {0};
        sprintf(line, "| TX frequency: %lu Hz", dev->subdevice[i].tx_frequency);
        for(uint8_t j=strlen(line); j<39; j++)
            line[j]=' ';
        sprintf(&line[strlen(line)], "|");

        dbg_print(TERM_DEFAULT, "%s", line);
    }
    printf("\n");    

    for(uint8_t i=0; i<num_devices; i++)
    {
        char line[100] = {0};
        sprintf(line, "| LNA gain: %2.2f dB", dev->subdevice[i].lna_gain);
        for(uint8_t j=strlen(line); j<39; j++)
            line[j]=' ';
        sprintf(&line[strlen(line)], "|");

        dbg_print(TERM_DEFAULT, "%s", line);
    }
    printf("\n");

    for(uint8_t i=0; i<num_devices; i++)
    {
        char line[100] = {0};
        sprintf(line, "| Power: %2.2f dBm", dev->subdevice[i].power);
        for(uint8_t j=strlen(line); j<39; j++)
            line[j]=' ';
        sprintf(&line[strlen(line)], "|");

        dbg_print(TERM_DEFAULT, "%s", line);
    }
    printf("\n");

    for(uint8_t i=0; i<num_devices; i++)
    {
        char line[100] = {0};
        sprintf(line, "| Channel width: %6.0f Hz", dev->subdevice[i].ch_width);
        for(uint8_t j=strlen(line); j<39; j++)
            line[j]=' ';
        sprintf(&line[strlen(line)], "|");

        dbg_print(TERM_DEFAULT, "%s", line);
    }
    printf("\n");

    for(uint8_t i=0; i<num_devices; i++)
    {
        char line[100] = {0};
        sprintf(line, "| Sample rate: %6.0f Hz", dev->subdevice[i].samp_rate);
        for(uint8_t j=strlen(line); j<39; j++)
            line[j]=' ';
        sprintf(&line[strlen(line)], "|");

        dbg_print(TERM_DEFAULT, "%s", line);
    }
    printf("\n");

    for(uint8_t i=0; i<num_devices; i++)
    {
        char line[100] = {0};
        sprintf(line, "| Frequency correction: %2.2f ppm", dev->subdevice[i].f_corr);
        for(uint8_t j=strlen(line); j<39; j++)
            line[j]=' ';
        sprintf(&line[strlen(line)], "|");

        dbg_print(TERM_DEFAULT, "%s", line);
    }
    printf("\n");

    dbg_print(TERM_DEFAULT, "--------------------------------------------------------------------------------\n");

    dbg_print(TERM_DEFAULT, "Last CARI data: ");
    for(uint8_t i=0; i<cari->zmq_byte_cnt && i<16; i++)
        dbg_print(TERM_YELLOW, "%02X ", cari->zmq_buff[i]);
    dbg_print(TERM_DEFAULT, "\n");
}

void cari_pong(void *cari_ctrl, uint32_t err)
{
    uint8_t rep[7];

    rep[0]=CMD_PING;
    *((uint16_t*)&rep[1])=7;
    *((uint32_t*)&rep[3])=err;

    zmq_send(cari_ctrl, rep, 7, ZMQ_DONTWAIT);
}

//the len parameter is the total length
void cari_reply_addr(void *cari_ctrl, cid_t cid, uint8_t addr, uint8_t *params, uint8_t len)
{
    uint8_t rep[len];

    rep[0]=cid;
    *((uint16_t*)&rep[1])=len;
    rep[3]=addr;
    memcpy(&rep[4], params, len-4);

    zmq_send(cari_ctrl, rep, len, ZMQ_DONTWAIT);    
}

//the len parameter is the total length
void cari_reply_noaddr(void *cari_ctrl, cid_t cid, uint8_t *params, uint8_t len)
{
    uint8_t rep[len+3];

    rep[0]=cid;
    *((uint16_t*)&rep[1])=len+3;
    memcpy(&rep[3], params, len);

    zmq_send(cari_ctrl, rep, len+3, ZMQ_DONTWAIT);    
}

int main(void)
{
    cari_init(&cari);

    dispSettings(&device, &cari);

    while(1)
    {
        //simple ACK echo for the CTRL plane
        cari.zmq_byte_cnt = zmq_recv(cari.zmq_ctrl, cari.zmq_buff, sizeof(cari.zmq_buff), ZMQ_DONTWAIT);

        if(cari.zmq_byte_cnt>0 && cari.zmq_byte_cnt==*((uint16_t*)&cari.zmq_buff[1]))
        {
            uint8_t cid = cari.zmq_buff[0];

            if(cid==CMD_PING) //PING
            {
                cari_pong(cari.zmq_ctrl, 0);
            }

            else if(cid==CMD_DEV_SET_REG)
            {
                uint8_t resp = CARI_OK;
                cari_reply_noaddr(cari.zmq_ctrl, cid, &resp, 1);
            }

            else if(cid==CMD_DEV_GET_IDENT)
            {
                cari_reply_noaddr(cari.zmq_ctrl, cid, (uint8_t*)device.ident, strlen(device.ident));
            }

            else if(cid==CMD_DEV_GET_REG)
            {
                uint8_t addr = cari.zmq_buff[3];
                uint8_t resp;

                if(addr==0)
                {
                    resp = CARI_VER;
                    cari_reply_noaddr(cari.zmq_ctrl, cid, &resp, 1);
                }
            }

            else //unrecognized command
            {
                uint8_t err = CARI_UNSUPPORTED;
                cari_reply_noaddr(cari.zmq_ctrl, cid, &err, 1);
            }

            dispSettings(&device, &cari);
        }
    }

    return 0;
}
