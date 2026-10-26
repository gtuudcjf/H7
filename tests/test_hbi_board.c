#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fmc.h"
#include "gpio.h"
#include "lan9252_board.h"
#include "mpu_body.h"

GPIO_TypeDef ports[7];
FakeDWT fake_dwt;
FakeCoreDebug fake_core_debug;
uint32_t SystemCoreClock=480000000U;
uint16_t hbi_memory[512];
static unsigned checks,dsbs,mpu_count;
static uint32_t tick;
static unsigned counter_stalled;
static MPU_Region_InitTypeDef regions[2];
#define CHECK(x) do { ++checks; if(!(x)) { fprintf(stderr,"board line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
void Error_Handler(void) { CHECK(0); }
void fake_dsb(void) { ++dsbs; }
void fake_nop(void)
{
    if(!counter_stalled && fake_dwt.LAR==0xc5acce55 && (fake_dwt.CTRL&1) &&
       (fake_core_debug.DEMCR&0x1000000)) ++fake_dwt.CYCCNT;
}
uint32_t HAL_GetTick(void) { return tick; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p,uint32_t pin,unsigned v)
{
    if((p==GPIOB && (pin&0x20)) || (p==GPIOC && (pin&0x10))) CHECK(v==0);
    if(v)p->odr|=pin; else p->odr&=~pin;
}
void HAL_GPIO_Init(GPIO_TypeDef *p,GPIO_InitTypeDef *c)
{
    unsigned i;
    if(c->Mode==GPIO_MODE_OUTPUT_PP) {
        if(p==GPIOB) CHECK((p->odr&0x20)==0);
        if(p==GPIOC) CHECK((p->odr&0x10)==0);
        if(p==GPIOF) CHECK((p->odr&2)==2);
    }
    for(i=0;i<16;++i) if(c->Pin&(1U<<i)) {
        p->mode[i]=c->Mode; p->alternate[i]=c->Alternate; p->pull[i]=c->Pull;
    }
}
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *c)
{ CHECK(c->PeriphClockSelection==1 && c->FmcClockSelection==0); return HAL_OK; }
HAL_StatusTypeDef HAL_SRAM_Init(SRAM_HandleTypeDef *h,FMC_NORSRAM_TimingTypeDef *r,FMC_NORSRAM_TimingTypeDef *w)
{
    CHECK(h->Instance==FMC_NORSRAM_DEVICE && h->Extended==FMC_NORSRAM_EXTENDED_DEVICE);
    CHECK(h->Init.NSBank==6 && h->Init.DataAddressMux==2 && h->Init.MemoryType==8 && h->Init.MemoryDataWidth==16);
    CHECK(h->Init.BurstAccessMode==0 && h->Init.WaitSignal==0 && h->Init.AsynchronousWait==0 && h->Init.WriteBurst==0);
    CHECK(h->Init.WriteFifo==2097152 && h->Init.WriteOperation==4096);
    CHECK(r->AddressSetupTime==15 && r->AddressHoldTime==15 && r->DataSetupTime==63 && r->BusTurnAroundDuration==15);
    CHECK(r==w && r->AccessMode==0);
    HAL_SRAM_MspInit(h); return HAL_OK;
}
void HAL_MPU_Disable(void) { mpu_count=0; }
void HAL_MPU_ConfigRegion(MPU_Region_InitTypeDef *r) { CHECK(mpu_count<2); regions[mpu_count++]=*r; }
void HAL_MPU_Enable(uint32_t flags) { CHECK(flags==4); }
int main(int argc,char **argv)
{
    const LAN9252Transport *t;
    uint16_t value=0;
    uint32_t start;
    unsigned p,i;
    memset(ports,0xff,sizeof(ports));
    MX_GPIO_Init(); MX_FMC_Init();
    CHECK(GPIOB->mode[5]==1 && (GPIOB->odr&0x20)==0);
    CHECK(GPIOC->mode[4]==1 && (GPIOC->odr&0x10)==0);
    CHECK(GPIOF->mode[1]==1 && (GPIOF->odr&2)==2);
    for(i=4;i<=7;++i) CHECK(GPIOA->mode[i]==3 && GPIOA->pull[i]==0);
    CHECK(GPIOF->mode[0]==3 && GPIOF->mode[2]==3 && GPIOF->mode[3]==3);
    CHECK(GPIOE->mode[0]==0 && GPIOE->mode[1]==0 && GPIOE->mode[3]==0);
    for(p=0;p<7;++p) for(i=0;i<16;++i) {
        unsigned expected=(p==1 && i==7) || (p==6 && i==12) ||
            (p==3 && ((0xc733U>>i)&1U)) || (p==4 && i>=7);
        if(expected) CHECK(ports[p].mode[i]==2 && ports[p].alternate[i]==12);
        else CHECK(ports[p].mode[i]!=2);
    }
    MPU_Config();
    CHECK(mpu_count==2 && regions[0].AccessPermission==0);
    CHECK(regions[1].Number==1 && regions[1].BaseAddress==0x6c000000 && regions[1].Size==15);
    CHECK(regions[1].SubRegionDisable==0 && regions[1].AccessPermission==3 && regions[1].DisableExec==1);
    CHECK(regions[1].TypeExtField==2 && regions[1].IsCacheable==0 && regions[1].IsBufferable==0);
    (void)argv;
    if(argc>1) counter_stalled=1;
    t=LAN9252_BoardTransport();
    if(counter_stalled) {
        CHECK(t==NULL);
        printf("test_hbi_board stalled counter: PASS\n"); return 0;
    }
    CHECK(t!=NULL && DWT->LAR==0xc5acce55);
    hbi_memory[0x32]=0x4321; hbi_memory[0x33]=0x8765;
    CHECK(t->read16(t->context,0x64,&value)==LAN9252_OK && value==0x4321);
    CHECK(t->read16(t->context,0x66,&value)==LAN9252_OK && value==0x8765);
    CHECK(t->write16(t->context,0x20,0xabcd)==LAN9252_OK && hbi_memory[0x10]==0xabcd);
    start=DWT->CYCCNT; t->finish_dword(t->context); CHECK((uint32_t)(DWT->CYCCNT-start)>=480);
    CHECK(dsbs>=5 && (DWT->CTRL&1) && (CoreDebug->DEMCR&0x1000000));
    CHECK(t->time_us(t->context)==1);
    DWT->CYCCNT+=480000; ++tick; CHECK(t->time_us(t->context)==1001);
    DWT->CYCCNT+=4800000U; tick+=10; CHECK(t->time_us(t->context)==11001);
    /* 10 second gap includes a full DWT wrap at 480MHz. */
    DWT->CYCCNT+=(uint32_t)UINT64_C(4800000000); tick+=10000;
    CHECK(t->time_us(t->context)==10011001);
    printf("test_hbi_board: PASS (%u checks)\n",checks); return 0;
}
