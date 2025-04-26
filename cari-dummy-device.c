/*
 * cari-dummy-device.c
 *
 *  Edited on: Jan 9, 2025
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
#define CARI_VER            (((uint8_t)1<<4) | 1)   //CARI v1.1
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
    SUB_RX_START,
    SUB_RX_STOP
} sub_act_t;

typedef struct
{
    uint16_t ctrl_port;
    int zmq_byte_cnt;
    void *zmq_ctx, *zmq_dlink, *zmq_meas, *zmq_ulink, *zmq_ctrl;
    uint8_t ctrl_ok;
    uint8_t zmq_buff[1024];
} cari_t;

//default settings
cari_t cari =
{
    .ctrl_port = 17001,
};

typedef struct
{
    uint64_t        frequency;              //Hz
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
    uint8_t cari_version;
    uint8_t num_capabilities;
    dev_cap_t capabilities[16];
    uint8_t subdevices;
    subdevice_t subdevice[];
} device_t;

//sample settings
device_t device =
{
    CARI_VER,             //CARI version 1.1
    1,                      //number of capabilities
    {DEV_SUPERVISION},      //supervision channel available
    2, {
    {438.8125e6, 0.0f, 30.0f, 12.5e3, 125e3, 0.0f, 5, {SUB_TX, SUB_FM_MOD, SUB_FM_DEMOD, SUB_AFC, SUB_AGC}},
    {431.2125e6, 0.0f, 30.0f, 12.5e3, 125e3, 0.0f, 5, {SUB_RX, SUB_FM_MOD, SUB_FM_DEMOD, SUB_AFC, SUB_AGC}}}
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

int cari_init(cari_t *cari)
{
    cari->zmq_ctx = zmq_ctx_new();
    cari->zmq_dlink = zmq_socket(cari->zmq_ctx, ZMQ_PUB);
	cari->zmq_meas = zmq_socket(cari->zmq_ctx, ZMQ_PUB);
	cari->zmq_ulink = zmq_socket(cari->zmq_ctx, ZMQ_SUB);
    cari->zmq_ctrl = zmq_socket(cari->zmq_ctx, ZMQ_REP);

    char addr[128];
    sprintf(addr, "tcp://*:%d", cari->ctrl_port);

    int res = zmq_bind(cari->zmq_ctrl, addr);

    if(res)
    {
        cari->ctrl_ok = 0;
    }
    else
    {
        cari->ctrl_ok = 1;
    }

    return res;
}

void dispSettings(device_t *dev, cari_t *cari)
{
    uint8_t num_devices = dev->subdevices;

    dbg_print(TERM_CLR, "");

    dbg_print(TERM_DEFAULT, "CARI dummy device\n");

    dbg_print(TERM_DEFAULT, "CARI version: ");
    dbg_print(TERM_YELLOW, "%d.%d\n", (dev->cari_version)>>4, (dev->cari_version)&0xF);

    dbg_print(TERM_DEFAULT, "CARI capabilities: ");
    for(uint8_t i=0; i<dev->num_capabilities; i++)
        dbg_print(TERM_YELLOW, "%02X " , dev->capabilities[i]);
    dbg_print(TERM_DEFAULT, "\n");

    dbg_print(TERM_DEFAULT, "CTRL port: ");
    dbg_print(TERM_YELLOW, "%d", cari->ctrl_port);
    dbg_print(TERM_DEFAULT, ", status ");
    if(cari->ctrl_ok)
        dbg_print(TERM_GREEN, "OK\n");
    else
        dbg_print(TERM_YELLOW, "IDLE\n");

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
        char line[100]; memset(line, 0, 100);
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
        char line[100]; memset(line, 0, 100);
        sprintf(line, "| Frequency: %lu Hz", dev->subdevice[i].frequency);
        for(uint8_t j=strlen(line); j<39; j++)
            line[j]=' ';
        sprintf(&line[strlen(line)], "|");

        dbg_print(TERM_DEFAULT, "%s", line);
    }
    printf("\n");

    for(uint8_t i=0; i<num_devices; i++)
    {
        char line[100]; memset(line, 0, 100);
        sprintf(line, "| LNA gain: %2.2f dB", dev->subdevice[i].lna_gain);
        for(uint8_t j=strlen(line); j<39; j++)
            line[j]=' ';
        sprintf(&line[strlen(line)], "|");

        dbg_print(TERM_DEFAULT, "%s", line);
    }
    printf("\n");

    for(uint8_t i=0; i<num_devices; i++)
    {
        char line[100]; memset(line, 0, 100);
        sprintf(line, "| Power: %2.2f dBm", dev->subdevice[i].power);
        for(uint8_t j=strlen(line); j<39; j++)
            line[j]=' ';
        sprintf(&line[strlen(line)], "|");

        dbg_print(TERM_DEFAULT, "%s", line);
    }
    printf("\n");

    for(uint8_t i=0; i<num_devices; i++)
    {
        char line[100]; memset(line, 0, 100);
        sprintf(line, "| Channel width: %6.0f Hz", dev->subdevice[i].ch_width);
        for(uint8_t j=strlen(line); j<39; j++)
            line[j]=' ';
        sprintf(&line[strlen(line)], "|");

        dbg_print(TERM_DEFAULT, "%s", line);
    }
    printf("\n");

    for(uint8_t i=0; i<num_devices; i++)
    {
        char line[100]; memset(line, 0, 100);
        sprintf(line, "| Sample rate: %6.0f Hz", dev->subdevice[i].samp_rate);
        for(uint8_t j=strlen(line); j<39; j++)
            line[j]=' ';
        sprintf(&line[strlen(line)], "|");

        dbg_print(TERM_DEFAULT, "%s", line);
    }
    printf("\n");

    for(uint8_t i=0; i<num_devices; i++)
    {
        char line[100]; memset(line, 0, 100);
        sprintf(line, "| Frequency correction: %2.2f ppm", dev->subdevice[i].f_corr);
        for(uint8_t j=strlen(line); j<39; j++)
            line[j]=' ';
        sprintf(&line[strlen(line)], "|");

        dbg_print(TERM_DEFAULT, "%s", line);
    }
    printf("\n");

    dbg_print(TERM_DEFAULT, "--------------------------------------------------------------------------------\n");

    dbg_print(TERM_DEFAULT, "Last CARI data: ");
    for(uint8_t i=0; i<cari->zmq_byte_cnt && i<8; i++)
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
    uint8_t rep[len];

    rep[0]=cid;
    *((uint16_t*)&rep[1])=len;
    memcpy(&rep[3], params, len-3);

    zmq_send(cari_ctrl, rep, len, ZMQ_DONTWAIT);    
}

int main(void)
{
    cari_init(&cari);

    dispSettings(&device, &cari);

    while(1)
    {
        //simple ACK echo for the CTRL plane
        cari.zmq_byte_cnt = zmq_recv(cari.zmq_ctrl, cari.zmq_buff, sizeof(cari.zmq_buff), ZMQ_DONTWAIT);

        if(cari.zmq_byte_cnt>0 /*&& cari.zmq_byte_cnt==*((uint16_t*)&cari.zmq_buff[1])*/)
        {
            uint8_t cid = cari.zmq_buff[0];

            if(cid==CMD_PING) //PING
            {
                cari_pong(cari.zmq_ctrl, 0);
            }

            else if(cid==CMD_DEV_GET_IDENT)
            {
                cari_reply_noaddr(cari.zmq_ctrl, CMD_DEV_GET_IDENT, (uint8_t*)CARI_DEV_IDENT, strlen(CARI_DEV_IDENT)+3);
            }

            else if(cid==CMD_DEV_GET_REG)
            {
                uint8_t resp = CARI_VER;
                cari_reply_noaddr(cari.zmq_ctrl, CMD_DEV_GET_REG, &resp, 4);
            }

            else //unrecognized command
            {
                uint8_t err = CARI_UNSUPPORTED;
                cari_reply_noaddr(cari.zmq_ctrl, cid, &err, 4);
            }

            dispSettings(&device, &cari);
        }
    }

    return 0;
}
