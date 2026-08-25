/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     80000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMA0
#define PWM_0_INST_IRQHandler                                   TIMA0_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA0_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                             80000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOA
#define GPIO_PWM_0_C0_PIN                                          DL_GPIO_PIN_8
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM19)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM19_PF_TIMA0_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOA
#define GPIO_PWM_0_C1_PIN                                          DL_GPIO_PIN_9
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM20)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM20_PF_TIMA0_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMG6)
#define TIMER_0_INST_IRQHandler                                 TIMG6_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG6_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                          (7999U)
/* Defines for NTB */
#define NTB_INST                                                        (TIMG12)
#define NTB_INST_IRQHandler                                    TIMG12_IRQHandler
#define NTB_INST_INT_IRQN                                      (TIMG12_INT_IRQn)
#define NTB_INST_LOAD_VALUE                                        (2999999999U)



/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_0_FBRD_40_MHZ_115200_BAUD                                      (45)
/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOB
#define GPIO_UART_1_TX_PORT                                                GPIOB
#define GPIO_UART_1_RX_PIN                                         DL_GPIO_PIN_5
#define GPIO_UART_1_TX_PIN                                         DL_GPIO_PIN_4
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM18)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM17)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM18_PF_UART1_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM17_PF_UART1_TX
#define UART_1_BAUD_RATE                                                (115200)
#define UART_1_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_1_FBRD_40_MHZ_115200_BAUD                                      (45)
/* Defines for UART_2 */
#define UART_2_INST                                                        UART2
#define UART_2_INST_IRQHandler                                  UART2_IRQHandler
#define UART_2_INST_INT_IRQN                                      UART2_INT_IRQn
#define GPIO_UART_2_RX_PORT                                                GPIOB
#define GPIO_UART_2_TX_PORT                                                GPIOB
#define GPIO_UART_2_RX_PIN                                        DL_GPIO_PIN_18
#define GPIO_UART_2_TX_PIN                                        DL_GPIO_PIN_15
#define GPIO_UART_2_IOMUX_RX                                     (IOMUX_PINCM44)
#define GPIO_UART_2_IOMUX_TX                                     (IOMUX_PINCM32)
#define GPIO_UART_2_IOMUX_RX_FUNC                      IOMUX_PINCM44_PF_UART2_RX
#define GPIO_UART_2_IOMUX_TX_FUNC                      IOMUX_PINCM32_PF_UART2_TX
#define UART_2_BAUD_RATE                                                  (9600)
#define UART_2_IBRD_40_MHZ_9600_BAUD                                       (260)
#define UART_2_FBRD_40_MHZ_9600_BAUD                                        (27)





/* Port definition for Pin Group Motor */
#define Motor_PORT                                                       (GPIOB)

/* Defines for AIN1: GPIOB.12 with pinCMx 29 on package pin 64 */
#define Motor_AIN1_PIN                                          (DL_GPIO_PIN_12)
#define Motor_AIN1_IOMUX                                         (IOMUX_PINCM29)
/* Defines for AIN2: GPIOB.13 with pinCMx 30 on package pin 1 */
#define Motor_AIN2_PIN                                          (DL_GPIO_PIN_13)
#define Motor_AIN2_IOMUX                                         (IOMUX_PINCM30)
/* Defines for BIN1: GPIOB.2 with pinCMx 15 on package pin 50 */
#define Motor_BIN1_PIN                                           (DL_GPIO_PIN_2)
#define Motor_BIN1_IOMUX                                         (IOMUX_PINCM15)
/* Defines for BIN2: GPIOB.3 with pinCMx 16 on package pin 51 */
#define Motor_BIN2_PIN                                           (DL_GPIO_PIN_3)
#define Motor_BIN2_IOMUX                                         (IOMUX_PINCM16)
/* Port definition for Pin Group ENCODERA */
#define ENCODERA_PORT                                                    (GPIOA)

/* Defines for E1A: GPIOA.22 with pinCMx 47 on package pin 18 */
// groups represented: ["ENCODERB","ENCODERA"]
// pins affected: ["E2A","E2B","E1A","E1B"]
#define GPIO_MULTIPLE_GPIOA_INT_IRQN                            (GPIOA_INT_IRQn)
#define GPIO_MULTIPLE_GPIOA_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define ENCODERA_E1A_IIDX                                   (DL_GPIO_IIDX_DIO22)
#define ENCODERA_E1A_PIN                                        (DL_GPIO_PIN_22)
#define ENCODERA_E1A_IOMUX                                       (IOMUX_PINCM47)
/* Defines for E1B: GPIOA.12 with pinCMx 34 on package pin 5 */
#define ENCODERA_E1B_IIDX                                   (DL_GPIO_IIDX_DIO12)
#define ENCODERA_E1B_PIN                                        (DL_GPIO_PIN_12)
#define ENCODERA_E1B_IOMUX                                       (IOMUX_PINCM34)
/* Port definition for Pin Group ENCODERB */
#define ENCODERB_PORT                                                    (GPIOA)

/* Defines for E2A: GPIOA.24 with pinCMx 54 on package pin 25 */
#define ENCODERB_E2A_IIDX                                   (DL_GPIO_IIDX_DIO24)
#define ENCODERB_E2A_PIN                                        (DL_GPIO_PIN_24)
#define ENCODERB_E2A_IOMUX                                       (IOMUX_PINCM54)
/* Defines for E2B: GPIOA.17 with pinCMx 39 on package pin 10 */
#define ENCODERB_E2B_IIDX                                   (DL_GPIO_IIDX_DIO17)
#define ENCODERB_E2B_PIN                                        (DL_GPIO_PIN_17)
#define ENCODERB_E2B_IOMUX                                       (IOMUX_PINCM39)
/* Defines for led: GPIOA.7 with pinCMx 14 on package pin 49 */
#define LED_led_PORT                                                     (GPIOA)
#define LED_led_PIN                                              (DL_GPIO_PIN_7)
#define LED_led_IOMUX                                            (IOMUX_PINCM14)
/* Defines for user: GPIOB.22 with pinCMx 50 on package pin 21 */
#define LED_user_PORT                                                    (GPIOB)
#define LED_user_PIN                                            (DL_GPIO_PIN_22)
#define LED_user_IOMUX                                           (IOMUX_PINCM50)
/* Defines for K1: GPIOB.27 with pinCMx 58 on package pin 29 */
#define KEY_K1_PORT                                                      (GPIOB)
#define KEY_K1_PIN                                              (DL_GPIO_PIN_27)
#define KEY_K1_IOMUX                                             (IOMUX_PINCM58)
/* Defines for K2: GPIOA.25 with pinCMx 55 on package pin 26 */
#define KEY_K2_PORT                                                      (GPIOA)
#define KEY_K2_PIN                                              (DL_GPIO_PIN_25)
#define KEY_K2_IOMUX                                             (IOMUX_PINCM55)
/* Defines for K3: GPIOA.26 with pinCMx 59 on package pin 30 */
#define KEY_K3_PORT                                                      (GPIOA)
#define KEY_K3_PIN                                              (DL_GPIO_PIN_26)
#define KEY_K3_IOMUX                                             (IOMUX_PINCM59)
/* Port definition for Pin Group OLED */
#define OLED_PORT                                                        (GPIOB)

/* Defines for SCL: GPIOB.9 with pinCMx 26 on package pin 61 */
#define OLED_SCL_PIN                                             (DL_GPIO_PIN_9)
#define OLED_SCL_IOMUX                                           (IOMUX_PINCM26)
/* Defines for SDA: GPIOB.8 with pinCMx 25 on package pin 60 */
#define OLED_SDA_PIN                                             (DL_GPIO_PIN_8)
#define OLED_SDA_IOMUX                                           (IOMUX_PINCM25)
/* Defines for RES: GPIOB.10 with pinCMx 27 on package pin 62 */
#define OLED_RES_PIN                                            (DL_GPIO_PIN_10)
#define OLED_RES_IOMUX                                           (IOMUX_PINCM27)
/* Defines for DC: GPIOB.11 with pinCMx 28 on package pin 63 */
#define OLED_DC_PIN                                             (DL_GPIO_PIN_11)
#define OLED_DC_IOMUX                                            (IOMUX_PINCM28)
/* Defines for CH0: GPIOB.24 with pinCMx 52 on package pin 23 */
#define TRACK_CH0_PORT                                                   (GPIOB)
#define TRACK_CH0_PIN                                           (DL_GPIO_PIN_24)
#define TRACK_CH0_IOMUX                                          (IOMUX_PINCM52)
/* Defines for CH1: GPIOB.25 with pinCMx 56 on package pin 27 */
#define TRACK_CH1_PORT                                                   (GPIOB)
#define TRACK_CH1_PIN                                           (DL_GPIO_PIN_25)
#define TRACK_CH1_IOMUX                                          (IOMUX_PINCM56)
/* Defines for CH2: GPIOB.20 with pinCMx 48 on package pin 19 */
#define TRACK_CH2_PORT                                                   (GPIOB)
#define TRACK_CH2_PIN                                           (DL_GPIO_PIN_20)
#define TRACK_CH2_IOMUX                                          (IOMUX_PINCM48)
/* Defines for CH3: GPIOA.14 with pinCMx 36 on package pin 7 */
#define TRACK_CH3_PORT                                                   (GPIOA)
#define TRACK_CH3_PIN                                           (DL_GPIO_PIN_14)
#define TRACK_CH3_IOMUX                                          (IOMUX_PINCM36)
/* Defines for CH4: GPIOA.16 with pinCMx 38 on package pin 9 */
#define TRACK_CH4_PORT                                                   (GPIOA)
#define TRACK_CH4_PIN                                           (DL_GPIO_PIN_16)
#define TRACK_CH4_IOMUX                                          (IOMUX_PINCM38)
/* Defines for CH5: GPIOB.17 with pinCMx 43 on package pin 14 */
#define TRACK_CH5_PORT                                                   (GPIOB)
#define TRACK_CH5_PIN                                           (DL_GPIO_PIN_17)
#define TRACK_CH5_IOMUX                                          (IOMUX_PINCM43)
/* Defines for CH6: GPIOB.19 with pinCMx 45 on package pin 16 */
#define TRACK_CH6_PORT                                                   (GPIOB)
#define TRACK_CH6_PIN                                           (DL_GPIO_PIN_19)
#define TRACK_CH6_IOMUX                                          (IOMUX_PINCM45)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_NTB_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_UART_2_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
