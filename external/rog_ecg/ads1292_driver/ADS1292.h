#ifndef ADS1292_H
#define	ADS1292_H
#include "stdbool.h"
#include "device_api.h"

#define MAX_BAG_NUM		80//75一包80个点
#define	MAX_BR_NUM		MAX_BAG_NUM/10

#define	ID_ADS1292R		0x73

#define	PIN_START		29
#define	PIN_PWDN		30	//RESET和PWDN同一个引脚
#define	PIN_DRDY		9

#define	SPI0_SS_PIN		28
#define	SPI0_MISO_PIN	pin_1292_miso
#define	SPI0_MOSI_PIN	pin_1292_mosi
#define	SPI0_SCK_PIN	pin_1292_sck

#define	Set_START_H		NRF_P0->OUTSET = (1<<PIN_START)
#define	Set_START_L		NRF_P0->OUTCLR = (1<<PIN_START)
#define	Set_PWDN_H		NRF_P0->OUTSET = (1<<PIN_PWDN)
#define	Set_PWDN_L		NRF_P0->OUTCLR = (1<<PIN_PWDN)


//ads1292命令定义
//系统命令
#define ADS_WAKEUP		0x02	//从待机模式唤醒
#define ADS_STANDBY		0x04	//进入待机模式
#define ADS_RESET		0x06	//复位器件
#define ADS_START		0x08	//启动/重新启动(同步)转换
#define ADS_STOP		0x0A	//停止转换
#define ADS_OFFSETCAL	0x1A	//通道偏移校准

//数据读取命令
#define RDATAC			0x10	//启用连续读取数据模式
#define SDATAC			0x11	//停止连续读取数据模式
#define RDATA			0x12	//通过命令读取数据；支持多个读回
//寄存器读取命令
#define	RREG			0x20
#define WREG			0x40

//寄存器
//器件设置（只读寄存器）
#define ADS_ID			0x00
//各个通道上的全局设置
#define ADS_CONFIG1		0x01
#define ADS_CONFIG2		0x02
#define	LOFF			0x03
//特定于通道的设置
#define CH1SET			0x04
#define CH2SET			0x05
#define RLD_SENS		0x06
#define LOFF_SENS		0x07
#define LOFF_STAT		0x08
#define RESP1			0x09
#define RESP2			0x0A
#define GPIO			0x0B


#pragma pack(1)
typedef struct REG_STRUCT
{
	uint8_t m_id;
	uint8_t m_CONFIG1;
	uint8_t m_CONFIG2;
	uint8_t m_LOFF;
	uint8_t m_CH1SET;
	uint8_t m_CH2SET;
	uint8_t m_RLD_SENS;
	uint8_t m_LOFF_SENS;
	uint8_t m_LOFF_STAT;
	uint8_t m_RESP1;
	uint8_t m_RESP2;
	uint8_t m_GPIO;
}REG_STRUCT_DEF;

typedef struct 
{
	uint8_t DH;		//高
	uint8_t DM;		//中
	uint8_t DL;		//低
}DataBag;



typedef struct ADS_DATA_Typedef
{
	DataBag STAT;
	DataBag CHn[2];
}ADS_DATA_Typedef;
#pragma pack()

void SPI0_init(void);
bool ADS1292_Init(void);
//bool ADS1292_LOFFSENS_debug(uint8_t loff);
void ADS_gpiote_init(void);
void ADS_READ_DATA_Continue(ADS_DATA_Typedef* DATA);
void START_ADS(void);
void Stop_ADS(void);

#endif
