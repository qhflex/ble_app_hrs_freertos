
// 初始化
void init()
{
    // 1292初始化
    SPI0_init();
    nrf_gpio_cfg_input(PIN_DRDY,NRF_GPIO_PIN_PULLUP);
    ADS_gpiote_init();
    for(i = 0;i < 5;i++)
    {
        ADS_INIT_Flag = ADS1292_Init();
        if(ADS_INIT_Flag)
        {
            START_ADS(); 
            break;
        }
        else
            nrf_delay_ms(30);
    }

    //算法初始化
    ECG_2D_initialize();
    BRHPFilter_initialize();
    resetQRSDetect(200);
}




// 数据接收，处理
// pGET_BLE_BAG->ECG.LOFF_STATE 为脱落状态
// pGET_BLE_BAG->BR.CH1 为CH1呼吸数据
// pGET_BLE_BAG->ECG.CH2 为CH2为心电数据
// get_hr 为计算的心率值

for (;;)
{

    if(ECG_BAG_Flag)//有心电数据
    {
        ECG_BAG_Flag = false;
        
        pGET_BLE_BAG->ECG.LOFF_STATE = ((pSend_ADS->STAT.DH<<1) | (pSend_ADS->STAT.DM>>7));
        
        for(i = 0,j = 0;i < MAX_BAG_NUM;i++)
        {
            if((i%10) == 0)
            {
                result = (pSend_ADS[i].CHn[0].DH<<24) | (pSend_ADS[i].CHn[0].DM<<16) | (pSend_ADS[i].CHn[0].DL<<8);
                In_Signal1 = (float)(result>>8);
                ECG_2D_step_1();
                BRHPFilter_U.Input = Out_Signal1;
                BRHPFilter_step();
                pGET_BLE_BAG->BR.CH1[j++] = (uint16_t)BRHPFilter_Y.Output;
            }
            result2 = (pSend_ADS[i].CHn[1].DH<<24) | (pSend_ADS[i].CHn[1].DM<<16) | (pSend_ADS[i].CHn[1].DL<<8);
            In_Signal2 = (float)(result2>>14);

            ECG_2D_step_2();		
            pGET_BLE_BAG->ECG.CH2[i] = (uint16_t)Out_Signal2;	
            
            
            static int QrsCount = 0;
            static int winPeak;
            static int filterData;
            static int sbPeak;
            static int get_hr;
            int delay;

            delay = QRSDetect(Out_Signal2, QrsCount, &winPeak, &filterData, &sbPeak, (int *)&get_hr);
            if (delay > 0) {
                QrsCount++;
            }
        }

    }

}