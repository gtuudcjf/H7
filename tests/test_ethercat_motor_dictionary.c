#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sdoserv.c"
#include "objdef.c"
#include "coeappl.c"
#include "ethercat_app.h"
#include "ethercat_motor_bridge.h"
UINT8 nAlStatus=STATE_OP;
UINT16 nPdInputSize=20,nPdOutputSize=8;
UINT16 u16SendMbxSize=128,u16ReceiveMbxSize=128;
BOOL bEcatInputUpdateRunning,bEcatOutputUpdateRunning,bEcatFirstOutputsReceived;
TMBX *pCoeSendStored;
static unsigned checks,received;
static uint8_t captured[8],access_fault;
static const uint8_t tx[20]={0x9a,0,0x78,0x56,0x34,0x12,0xce,0xff,0xd4,0xfe,0,0,0,0,0x64,0,0x44,0x33,0x22,0x11};
#define CHECK(x) do{++checks;if(!(x)){fprintf(stderr,"motor dictionary %d: %s\n",__LINE__,#x);exit(1);}}while(0)
BOOL HW_AccessFault(void){return access_fault;}
UINT8 MBX_MailboxSendReq(TMBX *p,UINT8 flags){(void)p;(void)flags;return 0;}
void EtherCATMotorBridge_SetLink(uint8_t active,uint8_t spi){(void)active;(void)spi;}
void EtherCATMotorBridge_Receive(const uint8_t *p,size_t n){CHECK(n==8);memcpy(captured,p,n);++received;}
void EtherCATMotorBridge_Pack(uint8_t *p){memcpy(p,tx,20);}
static uint32_t Read(uint16_t index,uint8_t sub,uint32_t size)
{
    uint16_t b[2]={0,0};uint32_t v=0;const TOBJECT *o=OBJ_GetObjectHandle(index);
    CHECK(o!=NULL);CHECK(OBJ_GetObjectLength(index,sub,o,0)==size);
    CHECK(OBJ_Read(index,sub,size,o,b,0)==0);memcpy(&v,b,size);return v;
}
int main(void)
{
    uint16_t in[10],out[4],szin=0,szout=0,data[2]={1,0};
    const uint8_t rx[8]={1,0,0xce,0xff,0x44,0x33,0x22,0x11};
    CHECK(MAX_PD_INPUT_SIZE==20 && MAX_PD_OUTPUT_SIZE==8);
    CHECK(REVISION_NUMBER==2);CHECK(COE_ObjDictionaryInit()==0);
    CHECK(APPL_GenerateMapping(&szin,&szout)==0 && szin==20 && szout==8);
    CHECK(Read(0x1600,0,1)==3);CHECK(Read(0x1600,2,4)==0x70000210);
    CHECK(Read(0x1a00,0,1)==8);CHECK(Read(0x1a00,8,4)==0x60000820);
    bEcatInputUpdateRunning=1;bEcatOutputUpdateRunning=1;bEcatFirstOutputsReceived=1;
    memcpy(out,rx,8);APPL_OutputMapping(out);CHECK(received==1 && memcmp(captured,rx,8)==0);
    APPL_Application();APPL_InputMapping(in);CHECK(memcmp(in,tx,20)==0);
    CHECK(Read(0x7000,2,2)==0xffce);CHECK(Read(0x7000,3,4)==0x11223344);
    CHECK(Read(0x6000,2,4)==0x12345678);CHECK(Read(0x6000,3,2)==0xffce);
    CHECK(Read(0x6000,8,4)==0x11223344);
    CHECK(OBJ_Write(0x7000,1,2,OBJ_GetObjectHandle(0x7000),data,0)==ABORTIDX_READ_ONLY_ENTRY);
    CHECK(OBJ_Write(0x7000,3,4,OBJ_GetObjectHandle(0x7000),data,0)==ABORTIDX_READ_ONLY_ENTRY);
    CHECK(received==1);
    bEcatOutputUpdateRunning=0;APPL_OutputMapping(out);CHECK(received==1);
    printf("motor real SSC dictionary: %u checks passed\n",checks);return 0;
}
