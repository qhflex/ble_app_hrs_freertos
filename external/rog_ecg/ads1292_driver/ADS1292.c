#include "stdint.h"
#include "ADS1292.h"
#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_gpiote.h"
#include "string.h"

#define	Continue_Read_Len	9//一次连续读的数据长度
#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi0 = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;

extern __no_init uint32_t noVola_ADSwflag;

REG_STRUCT_DEF gREG_ST = {
    0x00,   /* 0x00:    id */
    0x01,   /* 0x01:    CONFIG1 */
    0xE0,   /* 0x02:    CONFIG2 */
    0x10,   /* 0x03:    LOFF */
    0x81,   /* 0x04:    CH1SET */
    0x07,   /* 0x05:    CH2SET */
    0x20,   /* 0x06:    RLD_SENS */
    0x00,   /* 0x07:    LOFF_SENS */
    0x00,   /* 0x08:    LOFF_STAT */
    0x02,   /* 0x09:    RESP1 */
    0x01,   /* 0x0a:    RESP2 */
    0x0C    /* 0x0b:    GPIO */
};

ADS_DATA_Typedef st_ADS_DATA1[MAX_BAG_NUM];
ADS_DATA_Typedef st_ADS_DATA2[MAX_BAG_NUM];
ADS_DATA_Typedef *pGet_ADS;
ADS_DATA_Typedef *pSend_ADS;
volatile uint8_t Offset;
bool ECG_BAG_Flag = false;//true：ECG包准备好

static void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	//读数据
	ADS_READ_DATA_Continue(pGet_ADS+Offset);
	Offset++;
	if(Offset >= MAX_BAG_NUM)
	{
		Offset = 0;
		ECG_BAG_Flag = true;
		pSend_ADS = pGet_ADS;
		if(pGet_ADS == st_ADS_DATA1)
			pGet_ADS = st_ADS_DATA2;
		else
			pGet_ADS = st_ADS_DATA1;
	}
}

//按键初始化的时候已经初始化过了。
void ADS_gpiote_init(void)
{
	ret_code_t err_code;
	
	//初始化GPIOTE程序模块,
//	err_code = nrf_drv_gpiote_init();
//	APP_ERROR_CHECK(err_code);


	//以下代码配置PIN_DRDY作为GPIOTE输入，下降沿产生事件
	nrf_drv_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);

	//开启BUTTON4对应引脚的上拉电阻
	in_config.pull = NRF_GPIO_PIN_PULLUP;
	//配置引脚为GPIOTE输入，因为使用PPI连接，所以不需要注册事件回调函数
	err_code = nrfx_gpiote_in_init(PIN_DRDY, &in_config, in_pin_handler);
	APP_ERROR_CHECK(err_code);
	//使能该引脚所在GPIOTE通道的事件模式
	nrf_drv_gpiote_in_event_enable(PIN_DRDY, true);
	
}


void SPI0_init(void)
{
	spi_xfer_done = false;
	//初始化SPI引脚
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	spi_config.ss_pin   = SPI0_SS_PIN;
	spi_config.miso_pin = SPI0_MISO_PIN;
	spi_config.mosi_pin = SPI0_MOSI_PIN;
	spi_config.sck_pin  = SPI0_SCK_PIN;
	spi_config.frequency = NRF_DRV_SPI_FREQ_250K;//NRF_DRV_SPI_FREQ_250K;
	spi_config.mode = NRF_DRV_SPI_MODE_1;
	spi_config.irq_priority = 6;//6
	APP_ERROR_CHECK(nrf_drv_spi_init(&spi0, &spi_config, NULL, NULL));
	
}


void ADSPIN_init(void)
{
	//配置GPIO
	nrf_gpio_cfg_output(PIN_START);
	nrf_gpio_cfg_output(PIN_PWDN);//RESET
	Set_PWDN_H;//for reset	
	nrf_delay_ms(3);
	Set_PWDN_L;//reset
	nrf_delay_ms(100);
	Set_PWDN_H;//for reset	
	nrf_delay_ms(3);
	Set_START_L;
	
//	nrf_gpio_cfg_input(PIN_DRDY,NRF_GPIO_PIN_PULLUP);
//	ADS_gpiote_init();
}


void ADS_READ_DATA_Continue(ADS_DATA_Typedef* DATA)
{
//	uint8_t TX[Continue_Read_Len] = {0};
//	ADS_SPI_Transfer(TX,(uint8_t*)DATA,Continue_Read_Len);
//	spi_xfer_done = false;
	nrf_drv_spi_transfer(&spi0, NULL, 0, (uint8_t*)DATA, Continue_Read_Len);
//	while(spi_xfer_done == false);
}

void ADS_SEND_CMD(uint8_t cmd)
{
	uint8_t TX[1];
//	spi_xfer_done = false;
	TX[0] = cmd;
//	ADS_SPI_Transfer(TX,RX,1);
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, TX, 1, NULL, 0));
//	while(spi_xfer_done == false)
//	{
//		__WFE();
//	}
}

void ADS_RREG(REG_STRUCT_DEF *vREG_ST)
{
	uint8_t TX[14],RX[14];//28= sizeof(REG_STRUCT)+2
//	spi_xfer_done = false;
	TX[0] = 0x20;
	TX[1] = sizeof(REG_STRUCT_DEF)-1;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, TX, 2, RX, 14));
//	while(spi_xfer_done == false)
//	{
//		__WFE();
//	}
	memcpy((uint8_t*)vREG_ST,RX+2,sizeof(REG_STRUCT_DEF));//26=sizeof(REG_STRUCT)
}


void ADS_WREG(REG_STRUCT_DEF *vREG_ST)
{
	uint8_t TX[14];//28= sizeof(REG_STRUCT)+2
//	spi_xfer_done = false;
	TX[0] = 0x40;
	TX[1] = sizeof(REG_STRUCT_DEF)-1;
	memcpy(TX+2,(uint8_t*)vREG_ST,12);
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, TX, 14, NULL, 0));
//	while(spi_xfer_done == false)
//	{
//		__WFE();
//	}
}
void ADS_LOFFSENS_WREG(uint8_t vREG)
{
	uint8_t TX[14];//28= sizeof(REG_STRUCT)+2
//	spi_xfer_done = false;
	TX[0] = 0x47;   /* LOFF_SENS */
	TX[1] = 0;  //fixed 1byte
    TX[2] = vREG;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, TX, 3, NULL, 0));
//	while(spi_xfer_done == false)
//	{
//		__WFE();
//	}
}


//参考1292手册p63，内部时钟的启动方式
REG_STRUCT_DEF gregst;
bool ADS1292_Init(void)
{
//    REG_STRUCT_DEF gregst;
    
	pGet_ADS = st_ADS_DATA1;
	ADSPIN_init();
	ADS_SEND_CMD(SDATAC);
	nrf_delay_ms(5);    
	ADS_WREG(&gREG_ST);
	nrf_delay_ms(5);
    memset(&gregst,0,sizeof(gregst));
	//先判断ID
    ADS_RREG(&gregst);
	if(gregst.m_id == ID_ADS1292R)
	{
		return true;
	}
	else
		return false;
}

void START_ADS(void)
{
	Set_START_H;
	ADS_SEND_CMD(RDATAC);
}

void Stop_ADS(void)
{
	Set_START_L;
	ADS_SEND_CMD(SDATAC);
}
