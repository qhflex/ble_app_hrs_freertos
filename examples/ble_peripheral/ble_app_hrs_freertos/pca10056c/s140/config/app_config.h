#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

#define NRFX_CLOCK_CONFIG_LF_SRC                    0
#define CLOCK_CONFIG_LF_SRC                         0
#define CLOCK_CONFIG_LF_CAL_ENABLED                 1
#define NRF_SDH_CLOCK_LF_SRC                        0
#define NRF_SDH_CLOCK_LF_RC_CTIV                    16
#define NRF_SDH_CLOCK_LF_RC_TEMP_CTIV               2
#define NRF_SDH_CLOCK_LF_ACCURACY                   1


#define NRFX_POWER_ENABLED                          1
#define POWER_ENABLED                               1
#define NRF_PWR_MGMT_ENABLED                        1
#define NRF_PWR_MGMT_CONFIG_FPU_SUPPORT_ENABLED     1

// ==== TIMER ====
#define NRFX_TIMER_ENABLED                          1
#define NRFX_TIMER1_ENABLED                         1
#define NRFX_TIMER2_ENABLED                         1
#define TIMER_ENABLED                               1
#define TIMER1_ENABLED                              1
#define TIMER2_ENABLED                              1

// ====  SPI  ====
#define NRFX_SPIM_ENABLED                           1
#define NRFX_SPI_ENABLED                            1
#define SPI_ENABLED                                 1
#define SPI1_ENABLED                                1
#define SPI1_USE_EASY_DMA                           0
#define SPI2_ENABLED                                1
#define SPI2_USE_EASY_DMA                           0
#define NRF_SPI_MNGR_ENABLED                        1

// ====  TWI  ====
#define NRFX_TWIM_ENABLED                           1
#define NRFX_TWI_ENABLED                            1
#define TWI_ENABLED                                 1
#define TWI0_ENABLED                                1
#define TWI0_USE_EASY_DMA                           0
#define NRF_TWI_MNGR_ENABLED                        1

// ==== UARTE ====
#define NRFX_UARTE_ENABLED                          1
#define NRFX_UART_ENABLED                           1
#define UART_ENABLED                                1
#define UART0_ENABLED                               1
#define UART0_CONFIG_USE_EASY_DMA                   1
#define UART1_ENABLED                               1

// === USBDCDC ===
#define NRFX_USBD_ENABLED                           1
#define USBD_ENABLED                                1
#define APP_USBD_ENABLED                            1
#define APP_USBD_VID                                0x1915
#define APP_USBD_PID                                0x521A
#define APP_USBD_CDC_ACM_ENABLED                    1
#define APP_USBD_CONFIG_EVENT_QUEUE_ENABLE          1
#define APP_USBD_CONFIG_SOF_HANDLING_MODE           1
#define APP_USBD_STRING_SERIAL                      APP_USBD_STRING_DESC("700a0b1b0a64")

// ==== PRINT ====
#define RETARGET_ENABLED                            0

// ===== LOG =====
#define NRF_LOG_BACKEND_UART_ENABLED                1
#define NRF_LOG_BACKEND_UART_TX_PIN                 16
#define NRF_LOG_BACKEND_UART_TEMP_BUFFER_SIZE       1024

// == NRF QUEUE ==
// required by twi manager
#define NRF_QUEUE_ENABLED                           1

// == HARDFAULT ==
#define HARDFAULT_HANDLER_ENABLED                   1

// = SDH DISPATCH = 
// - 2 for POLLING, which is required by freertos
#define NRF_SDH_DISPATCH_MODEL 2

// donno why enabled in template
#define NRF_SORTLIST_ENABLED                        0
#define APP_SCHEDULER_ENABLED                       0
#define APP_TIMER_CONFIG_RTC_FREQUENCY              0

#endif
