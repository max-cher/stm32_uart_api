/*******************************************************************************
*
* File Name          : stm32f10x.h
* Author             : MCD Application Team, Paul Robson
* Version            : V2.0
* Date               : 20/09/2010
* Description        : Includes as needed for STM32VLDISCOVERY coding.
*					   Built from _type.h and _map.h and simplified.
* Original Copyright : (c) 2007 ST Microelectronics
*
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifndef __STM32F10x_H
#define __STM32F10x_H

/******************************************************************************/
/*								Data Types Needed							  */
/******************************************************************************/

typedef signed long  s32;
typedef signed short s16;
typedef signed char  s8;

typedef signed long  const sc32;  /* Read Only */
typedef signed short const sc16;  /* Read Only */
typedef signed char  const sc8;   /* Read Only */

typedef volatile signed long  vs32;
typedef volatile signed short vs16;
typedef volatile signed char  vs8;

typedef volatile signed long  const vsc32;  /* Read Only */
typedef volatile signed short const vsc16;  /* Read Only */
typedef volatile signed char  const vsc8;   /* Read Only */

typedef unsigned long  u32;
typedef unsigned short u16;
typedef unsigned char  u8;

typedef unsigned long  const uc32;  /* Read Only */
typedef unsigned short const uc16;  /* Read Only */
typedef unsigned char  const uc8;   /* Read Only */

typedef volatile unsigned long  vu32;
typedef volatile unsigned short vu16;
typedef volatile unsigned char  vu8;

typedef volatile unsigned long  const vuc32;  /* Read Only */
typedef volatile unsigned short const vuc16;  /* Read Only */
typedef volatile unsigned char  const vuc8;   /* Read Only */

/******************************************************************************/
/*                         Peripheral registers structures                    */
/******************************************************************************/

/*------------------------ Analog to Digital Converter -----------------------*/
typedef struct
{
  vu32 SR;
  vu32 CR1;
  vu32 CR2;
  vu32 SMPR1;
  vu32 SMPR2;
  vu32 JOFR1;
  vu32 JOFR2;
  vu32 JOFR3;
  vu32 JOFR4;
  vu32 HTR;
  vu32 LTR;
  vu32 SQR1;
  vu32 SQR2;
  vu32 SQR3;
  vu32 JSQR;
  vu32 JDR1;
  vu32 JDR2;
  vu32 JDR3;
  vu32 JDR4;
  vu32 DR;
} ADC_TypeDef;

/*------------------------ Backup Registers ----------------------------------*/
typedef struct
{
  u32 RESERVED0;
  vu16 DR1;
  u16  RESERVED1;
  vu16 DR2;
  u16  RESERVED2;
  vu16 DR3;
  u16  RESERVED3;
  vu16 DR4;
  u16  RESERVED4;
  vu16 DR5;
  u16  RESERVED5;
  vu16 DR6;
  u16  RESERVED6;
  vu16 DR7;
  u16  RESERVED7;
  vu16 DR8;
  u16  RESERVED8;
  vu16 DR9;
  u16  RESERVED9;
  vu16 DR10;
  u16  RESERVED10;
  vu16 RTCCR;
  u16  RESERVED11;
  vu16 CR;
  u16  RESERVED12;
  vu16 CSR;
  u16  RESERVED13;
} BKP_TypeDef;

/*------------------------ Controller Area Network ---------------------------*/
typedef struct
{
  vu32 TIR;
  vu32 TDTR;
  vu32 TDLR;
  vu32 TDHR;
} CAN_TxMailBox_TypeDef;

typedef struct
{
  vu32 RIR;
  vu32 RDTR;
  vu32 RDLR;
  vu32 RDHR;
} CAN_FIFOMailBox_TypeDef;

typedef struct
{
  vu32 FR0;
  vu32 FR1;
} CAN_FilterRegister_TypeDef;

typedef struct
{
  vu32 MCR;
  vu32 MSR;
  vu32 TSR;
  vu32 RF0R;
  vu32 RF1R;
  vu32 IER;
  vu32 ESR;
  vu32 BTR;
  u32 RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  u32 RESERVED1[12];
  vu32 FMR;
  vu32 FM0R;
  u32 RESERVED2[1];
  vu32 FS0R;
  u32 RESERVED3[1];
  vu32 FFA0R;
  u32 RESERVED4[1];
  vu32 FA0R;
  u32 RESERVED5[8];
  CAN_FilterRegister_TypeDef sFilterRegister[14];
} CAN_TypeDef;

/*------------------------ DMA Controller ------------------------------------*/
typedef struct
{
  vu32 CCR;
  vu32 CNDTR;
  vu32 CPAR;
  vu32 CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  vu32 ISR;
  vu32 IFCR;
} DMA_TypeDef;

/*------------------------ External Interrupt/Event Controller ---------------*/
typedef struct
{
  vu32 IMR;
  vu32 EMR;
  vu32 RTSR;
  vu32 FTSR;
  vu32 SWIER;
  vu32 PR;
} EXTI_TypeDef;

/*------------------------ FLASH and Option Bytes Registers ------------------*/
typedef struct
{
  vu32 ACR;
  vu32 KEYR;
  vu32 OPTKEYR;
  vu32 SR;
  vu32 CR;
  vu32 AR;
  vu32 RESERVED;
  vu32 OBR;
  vu32 WRPR;
} FLASH_TypeDef;

typedef struct
{
  vu16 RDP;
  vu16 USER;
  vu16 Data0;
  vu16 Data1;
  vu16 WRP0;
  vu16 WRP1;
  vu16 WRP2;
  vu16 WRP3;
} OB_TypeDef;

/*------------------------ General Purpose and Alternate Function IO ---------*/
typedef struct
{
  vu32 CRL;
  vu32 CRH;
  vu32 IDR;
  vu32 ODR;
  vu32 BSRR;
  vu32 BRR;
  vu32 LCKR;
} GPIO_TypeDef;

typedef struct
{
  vu32 EVCR;
  vu32 MAPR;
  vu32 EXTICR[4];
} AFIO_TypeDef;

/*------------------------ Inter-integrated Circuit Interface ----------------*/
typedef struct
{
  vu16 CR1;
  u16 RESERVED0;
  vu16 CR2;
  u16 RESERVED1;
  vu16 OAR1;
  u16 RESERVED2;
  vu16 OAR2;
  u16 RESERVED3;
  vu16 DR;
  u16 RESERVED4;
  vu16 SR1;
  u16 RESERVED5;
  vu16 SR2;
  u16 RESERVED6;
  vu16 CCR;
  u16 RESERVED7;
  vu16 TRISE;
  u16 RESERVED8;
} I2C_TypeDef;

/*------------------------ Independent WATCHDOG ------------------------------*/
typedef struct
{
  vu32 KR;
  vu32 PR;
  vu32 RLR;
  vu32 SR;
} IWDG_TypeDef;

/*------------------------ Nested Vectored Interrupt Controller --------------*/
typedef struct
{
  vu32 ISER[2];
  u32 RESERVED0[30];
  vu32 ICER[2];
  u32 RSERVED1[30];
  vu32 ISPR[2];
  u32 RESERVED2[30];
  vu32 ICPR[2];
  u32 RESERVED3[30];
  vu32 IABR[2];
  u32 RESERVED4[62];
  vu32 IPR[11];
} NVIC_TypeDef;

typedef struct
{
  vuc32 CPUID;
  vu32 ICSR;
  vu32 VTOR;
  vu32 AIRCR;
  vu32 SCR;
  vu32 CCR;
  vu32 SHPR[3];
  vu32 SHCSR;
  vu32 CFSR;
  vu32 HFSR;
  vu32 DFSR;
  vu32 MMFAR;
  vu32 BFAR;
  vu32 AFSR;
} SCB_TypeDef;

/*------------------------ Power Control -------------------------------------*/
typedef struct
{
  vu32 CR;
  vu32 CSR;
} PWR_TypeDef;

/*------------------------ Reset and Clock Control ---------------------------*/
typedef struct
{
  vu32 CR;
  vu32 CFGR;
  vu32 CIR;
  vu32 APB2RSTR;
  vu32 APB1RSTR;
  vu32 AHBENR;
  vu32 APB2ENR;
  vu32 APB1ENR;
  vu32 BDCR;
  vu32 CSR;
} RCC_TypeDef;

/*------------------------ Real-Time Clock -----------------------------------*/
typedef struct
{
  vu16 CRH;
  u16 RESERVED0;
  vu16 CRL;
  u16 RESERVED1;
  vu16 PRLH;
  u16 RESERVED2;
  vu16 PRLL;
  u16 RESERVED3;
  vu16 DIVH;
  u16 RESERVED4;
  vu16 DIVL;
  u16 RESERVED5;
  vu16 CNTH;
  u16 RESERVED6;
  vu16 CNTL;
  u16 RESERVED7;
  vu16 ALRH;
  u16 RESERVED8;
  vu16 ALRL;
  u16 RESERVED9;
} RTC_TypeDef;

/*------------------------ Serial Peripheral Interface -----------------------*/
typedef struct
{
  vu16 CR1;
  u16 RESERVED0;
  vu16 CR2;
  u16 RESERVED1;
  vu16 SR;
  u16  RESERVED2;
  vu16 DR;
  u16  RESERVED3;
  vu16 CRCPR;
  u16 RESERVED4;
  vu16 RXCRCR;
  u16  RESERVED5;
  vu16 TXCRCR;
  u16  RESERVED6;
} SPI_TypeDef;

/*------------------------ SystemTick ----------------------------------------*/
typedef struct
{
  vu32 CTRL;
  vu32 LOAD;
  vu32 VAL;
  vuc32 CALIB;
} SysTick_TypeDef;

/*------------------------ Advanced Control Timer ----------------------------*/
typedef struct
{
  vu16 CR1;
  u16 RESERVED0;
  vu16 CR2;
  u16 RESERVED1;
  vu16 SMCR;
  u16 RESERVED2;
  vu16 DIER;
  u16 RESERVED3;
  vu16 SR;
  u16 RESERVED4;
  vu16 EGR;
  u16 RESERVED5;
  vu16 CCMR1;
  u16 RESERVED6;
  vu16 CCMR2;
  u16 RESERVED7;
  vu16 CCER;
  u16 RESERVED8;
  vu16 CNT;
  u16 RESERVED9;
  vu16 PSC;
  u16 RESERVED10;
  vu16 ARR;
  u16 RESERVED11;
  vu16 RCR;
  u16 RESERVED12;
  vu16 CCR1;
  u16 RESERVED13;
  vu16 CCR2;
  u16 RESERVED14;
  vu16 CCR3;
  u16 RESERVED15;
  vu16 CCR4;
  u16 RESERVED16;
  vu16 BDTR;
  u16 RESERVED17;
  vu16 DCR;
  u16 RESERVED18;
  vu16 DMAR;
  u16 RESERVED19;
} TIM1_TypeDef;

/*------------------------ General Purpose Timer -----------------------------*/
typedef struct
{
  vu16 CR1;
  u16 RESERVED0;
  vu16 CR2;
  u16 RESERVED1;
  vu16 SMCR;
  u16 RESERVED2;
  vu16 DIER;
  u16 RESERVED3;
  vu16 SR;
  u16 RESERVED4;
  vu16 EGR;
  u16 RESERVED5;
  vu16 CCMR1;
  u16 RESERVED6;
  vu16 CCMR2;
  u16 RESERVED7;
  vu16 CCER;
  u16 RESERVED8;
  vu16 CNT;
  u16 RESERVED9;
  vu16 PSC;
  u16 RESERVED10;
  vu16 ARR;
  u16 RESERVED11[3];
  vu16 CCR1;
  u16 RESERVED12;
  vu16 CCR2;
  u16 RESERVED13;
  vu16 CCR3;
  u16 RESERVED14;
  vu16 CCR4;
  u16 RESERVED15[3];
  vu16 DCR;
  u16 RESERVED16;
  vu16 DMAR;
  u16 RESERVED17;
} TIM_TypeDef;

/*----------------- Universal Synchronous Asynchronous Receiver Transmitter --*/
typedef struct
{
  vu16 SR;
  u16 RESERVED0;
  vu16 DR;
  u16 RESERVED1;
  vu16 BRR;
  u16 RESERVED2;
  vu16 CR1;
  u16 RESERVED3;
  vu16 CR2;
  u16 RESERVED4;
  vu16 CR3;
  u16 RESERVED5;
  vu16 GTPR;
  u16 RESERVED6;
} USART_TypeDef;

/*------------------------ Window WATCHDOG -----------------------------------*/
typedef struct
{
  vu32 CR;
  vu32 CFR;
  vu32 SR;
} WWDG_TypeDef;


/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/* Peripheral and SRAM base address in the alias region */
#define PERIPH_BB_BASE        ((u32)0x42000000)
#define SRAM_BB_BASE          ((u32)0x22000000)

/* Peripheral and SRAM base address in the bit-band region */
#define SRAM_BASE             ((u32)0x20000000)
#define PERIPH_BASE           ((u32)0x40000000)

/* Flash refisters base address */
#define FLASH_BASE            ((u32)0x40022000)
/* Flash Option Bytes base address */
#define OB_BASE               ((u32)0x1FFFF800)

/* Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x20000)

#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800)
#define CAN_BASE              (APB1PERIPH_BASE + 0x6400)
#define BKP_BASE              (APB1PERIPH_BASE + 0x6C00)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000)

#define AFIO_BASE             (APB2PERIPH_BASE + 0x0000)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x0400)
#define GPIOA_BASE            (APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASE            (APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASE            (APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASE            (APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASE            (APB2PERIPH_BASE + 0x1800)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2400)
#define ADC2_BASE             (APB2PERIPH_BASE + 0x2800)
#define TIM1_BASE             (APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000)
#define USART1_BASE           (APB2PERIPH_BASE + 0x3800)

#define DMA_BASE              (AHBPERIPH_BASE + 0x0000)
#define DMA_Channel1_BASE     (AHBPERIPH_BASE + 0x0008)
#define DMA_Channel2_BASE     (AHBPERIPH_BASE + 0x001C)
#define DMA_Channel3_BASE     (AHBPERIPH_BASE + 0x0030)
#define DMA_Channel4_BASE     (AHBPERIPH_BASE + 0x0044)
#define DMA_Channel5_BASE     (AHBPERIPH_BASE + 0x0058)
#define DMA_Channel6_BASE     (AHBPERIPH_BASE + 0x006C)
#define DMA_Channel7_BASE     (AHBPERIPH_BASE + 0x0080)
#define RCC_BASE              (AHBPERIPH_BASE + 0x1000)

/* System Control Space memory map */
#define SCS_BASE              ((u32)0xE000E000)

#define SysTick_BASE          (SCS_BASE + 0x0010)
#define NVIC_BASE             (SCS_BASE + 0x0100)
#define SCB_BASE              (SCS_BASE + 0x0D00)


/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/

#define TIM1                ((TIM1_TypeDef *) TIM1_BASE)
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)

#define RTC                 ((RTC_TypeDef *) RTC_BASE)

#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)

#define USART2              ((USART_TypeDef *) USART2_BASE)
#define USART3              ((USART_TypeDef *) USART3_BASE)

#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)

#define CAN                 ((CAN_TypeDef *) CAN_BASE)

#define BKP                 ((BKP_TypeDef *) BKP_BASE)

#define PWR                 ((PWR_TypeDef *) PWR_BASE)

#define AFIO                ((AFIO_TypeDef *) AFIO_BASE)

#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)

#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)

#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define ADC2                ((ADC_TypeDef *) ADC2_BASE)


#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)

#define USART1              ((USART_TypeDef *) USART1_BASE)

#define DMA                 ((DMA_TypeDef *) DMA_BASE)
#define DMA_Channel1        ((DMA_Channel_TypeDef *) DMA_Channel1_BASE)
#define DMA_Channel2        ((DMA_Channel_TypeDef *) DMA_Channel2_BASE)
#define DMA_Channel3        ((DMA_Channel_TypeDef *) DMA_Channel3_BASE)
#define DMA_Channel4        ((DMA_Channel_TypeDef *) DMA_Channel4_BASE)
#define DMA_Channel5        ((DMA_Channel_TypeDef *) DMA_Channel5_BASE)
#define DMA_Channel6        ((DMA_Channel_TypeDef *) DMA_Channel6_BASE)
#define DMA_Channel7        ((DMA_Channel_TypeDef *) DMA_Channel7_BASE)

#define FLASH               ((FLASH_TypeDef *) FLASH_BASE)
#define OB                  ((OB_TypeDef *) OB_BASE) 

#define RCC                 ((RCC_TypeDef *) RCC_BASE)

#define SysTick             ((SysTick_TypeDef *) SysTick_BASE)

#define NVIC                ((NVIC_TypeDef *) NVIC_BASE)
#define SCB                 ((SCB_TypeDef *) SCB_BASE)  


#define uint32_t u32
#define uint16_t u16

#define STM32F10X_CL


/******************************************************************************/
/*                                                                            */
/*         Universal Synchronous Asynchronous Receiver Transmitter            */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for USART_SR register  *******************/
#define  USART_SR_PE                         ((uint16_t)0x0001)            /*!<Parity Error */
#define  USART_SR_FE                         ((uint16_t)0x0002)            /*!<Framing Error */
#define  USART_SR_NE                         ((uint16_t)0x0004)            /*!<Noise Error Flag */
#define  USART_SR_ORE                        ((uint16_t)0x0008)            /*!<OverRun Error */
#define  USART_SR_IDLE                       ((uint16_t)0x0010)            /*!<IDLE line detected */
#define  USART_SR_RXNE                       ((uint16_t)0x0020)            /*!<Read Data Register Not Empty */
#define  USART_SR_TC                         ((uint16_t)0x0040)            /*!<Transmission Complete */
#define  USART_SR_TXE                        ((uint16_t)0x0080)            /*!<Transmit Data Register Empty */
#define  USART_SR_LBD                        ((uint16_t)0x0100)            /*!<LIN Break Detection Flag */
#define  USART_SR_CTS                        ((uint16_t)0x0200)            /*!<CTS Flag */

/*******************  Bit definition for USART_DR register  *******************/
#define  USART_DR_DR                         ((uint16_t)0x01FF)            /*!<Data value */

/******************  Bit definition for USART_BRR register  *******************/
#define  USART_BRR_DIV_Fraction              ((uint16_t)0x000F)            /*!<Fraction of USARTDIV */
#define  USART_BRR_DIV_Mantissa              ((uint16_t)0xFFF0)            /*!<Mantissa of USARTDIV */

/******************  Bit definition for USART_CR1 register  *******************/
#define  USART_CR1_SBK                       ((uint16_t)0x0001)            /*!<Send Break */
#define  USART_CR1_RWU                       ((uint16_t)0x0002)            /*!<Receiver wakeup */
#define  USART_CR1_RE                        ((uint16_t)0x0004)            /*!<Receiver Enable */
#define  USART_CR1_TE                        ((uint16_t)0x0008)            /*!<Transmitter Enable */
#define  USART_CR1_IDLEIE                    ((uint16_t)0x0010)            /*!<IDLE Interrupt Enable */
#define  USART_CR1_RXNEIE                    ((uint16_t)0x0020)            /*!<RXNE Interrupt Enable */
#define  USART_CR1_TCIE                      ((uint16_t)0x0040)            /*!<Transmission Complete Interrupt Enable */
#define  USART_CR1_TXEIE                     ((uint16_t)0x0080)            /*!<PE Interrupt Enable */
#define  USART_CR1_PEIE                      ((uint16_t)0x0100)            /*!<PE Interrupt Enable */
#define  USART_CR1_PS                        ((uint16_t)0x0200)            /*!<Parity Selection */
#define  USART_CR1_PCE                       ((uint16_t)0x0400)            /*!<Parity Control Enable */
#define  USART_CR1_WAKE                      ((uint16_t)0x0800)            /*!<Wakeup method */
#define  USART_CR1_M                         ((uint16_t)0x1000)            /*!<Word length */
#define  USART_CR1_UE                        ((uint16_t)0x2000)            /*!<USART Enable */
#define  USART_CR1_OVER8                     ((uint16_t)0x8000)            /*!<USART Oversmapling 8-bits */

/******************  Bit definition for USART_CR2 register  *******************/
#define  USART_CR2_ADD                       ((uint16_t)0x000F)            /*!<Address of the USART node */
#define  USART_CR2_LBDL                      ((uint16_t)0x0020)            /*!<LIN Break Detection Length */
#define  USART_CR2_LBDIE                     ((uint16_t)0x0040)            /*!<LIN Break Detection Interrupt Enable */
#define  USART_CR2_LBCL                      ((uint16_t)0x0100)            /*!<Last Bit Clock pulse */
#define  USART_CR2_CPHA                      ((uint16_t)0x0200)            /*!<Clock Phase */
#define  USART_CR2_CPOL                      ((uint16_t)0x0400)            /*!<Clock Polarity */
#define  USART_CR2_CLKEN                     ((uint16_t)0x0800)            /*!<Clock Enable */

#define  USART_CR2_STOP                      ((uint16_t)0x3000)            /*!<STOP[1:0] bits (STOP bits) */
#define  USART_CR2_STOP_0                    ((uint16_t)0x1000)            /*!<Bit 0 */
#define  USART_CR2_STOP_1                    ((uint16_t)0x2000)            /*!<Bit 1 */

#define  USART_CR2_LINEN                     ((uint16_t)0x4000)            /*!<LIN mode enable */

/******************  Bit definition for USART_CR3 register  *******************/
#define  USART_CR3_EIE                       ((uint16_t)0x0001)            /*!<Error Interrupt Enable */
#define  USART_CR3_IREN                      ((uint16_t)0x0002)            /*!<IrDA mode Enable */
#define  USART_CR3_IRLP                      ((uint16_t)0x0004)            /*!<IrDA Low-Power */
#define  USART_CR3_HDSEL                     ((uint16_t)0x0008)            /*!<Half-Duplex Selection */
#define  USART_CR3_NACK                      ((uint16_t)0x0010)            /*!<Smartcard NACK enable */
#define  USART_CR3_SCEN                      ((uint16_t)0x0020)            /*!<Smartcard mode enable */
#define  USART_CR3_DMAR                      ((uint16_t)0x0040)            /*!<DMA Enable Receiver */
#define  USART_CR3_DMAT                      ((uint16_t)0x0080)            /*!<DMA Enable Transmitter */
#define  USART_CR3_RTSE                      ((uint16_t)0x0100)            /*!<RTS Enable */
#define  USART_CR3_CTSE                      ((uint16_t)0x0200)            /*!<CTS Enable */
#define  USART_CR3_CTSIE                     ((uint16_t)0x0400)            /*!<CTS Interrupt Enable */
#define  USART_CR3_ONEBIT                    ((uint16_t)0x0800)            /*!<One Bit method */

/******************  Bit definition for USART_GTPR register  ******************/
#define  USART_GTPR_PSC                      ((uint16_t)0x00FF)            /*!<PSC[7:0] bits (Prescaler value) */
#define  USART_GTPR_PSC_0                    ((uint16_t)0x0001)            /*!<Bit 0 */
#define  USART_GTPR_PSC_1                    ((uint16_t)0x0002)            /*!<Bit 1 */
#define  USART_GTPR_PSC_2                    ((uint16_t)0x0004)            /*!<Bit 2 */
#define  USART_GTPR_PSC_3                    ((uint16_t)0x0008)            /*!<Bit 3 */
#define  USART_GTPR_PSC_4                    ((uint16_t)0x0010)            /*!<Bit 4 */
#define  USART_GTPR_PSC_5                    ((uint16_t)0x0020)            /*!<Bit 5 */
#define  USART_GTPR_PSC_6                    ((uint16_t)0x0040)            /*!<Bit 6 */
#define  USART_GTPR_PSC_7                    ((uint16_t)0x0080)            /*!<Bit 7 */

#define  USART_GTPR_GT                       ((uint16_t)0xFF00)            /*!<Guard time value */



/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for RCC_CR register  ********************/
#define  RCC_CR_HSION                        ((uint32_t)0x00000001)        /*!< Internal High Speed clock enable */
#define  RCC_CR_HSIRDY                       ((uint32_t)0x00000002)        /*!< Internal High Speed clock ready flag */
#define  RCC_CR_HSITRIM                      ((uint32_t)0x000000F8)        /*!< Internal High Speed clock trimming */
#define  RCC_CR_HSICAL                       ((uint32_t)0x0000FF00)        /*!< Internal High Speed clock Calibration */
#define  RCC_CR_HSEON                        ((uint32_t)0x00010000)        /*!< External High Speed clock enable */
#define  RCC_CR_HSERDY                       ((uint32_t)0x00020000)        /*!< External High Speed clock ready flag */
#define  RCC_CR_HSEBYP                       ((uint32_t)0x00040000)        /*!< External High Speed clock Bypass */
#define  RCC_CR_CSSON                        ((uint32_t)0x00080000)        /*!< Clock Security System enable */
#define  RCC_CR_PLLON                        ((uint32_t)0x01000000)        /*!< PLL enable */
#define  RCC_CR_PLLRDY                       ((uint32_t)0x02000000)        /*!< PLL clock ready flag */

#ifdef STM32F10X_CL
 #define  RCC_CR_PLL2ON                       ((uint32_t)0x04000000)        /*!< PLL2 enable */
 #define  RCC_CR_PLL2RDY                      ((uint32_t)0x08000000)        /*!< PLL2 clock ready flag */
 #define  RCC_CR_PLL3ON                       ((uint32_t)0x10000000)        /*!< PLL3 enable */
 #define  RCC_CR_PLL3RDY                      ((uint32_t)0x20000000)        /*!< PLL3 clock ready flag */
#endif /* STM32F10X_CL */

/*******************  Bit definition for RCC_CFGR register  *******************/
/*!< SW configuration */
#define  RCC_CFGR_SW                         ((uint32_t)0x00000003)        /*!< SW[1:0] bits (System clock Switch) */
#define  RCC_CFGR_SW_0                       ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  RCC_CFGR_SW_1                       ((uint32_t)0x00000002)        /*!< Bit 1 */

#define  RCC_CFGR_SW_HSI                     ((uint32_t)0x00000000)        /*!< HSI selected as system clock */
#define  RCC_CFGR_SW_HSE                     ((uint32_t)0x00000001)        /*!< HSE selected as system clock */
#define  RCC_CFGR_SW_PLL                     ((uint32_t)0x00000002)        /*!< PLL selected as system clock */

/*!< SWS configuration */
#define  RCC_CFGR_SWS                        ((uint32_t)0x0000000C)        /*!< SWS[1:0] bits (System Clock Switch Status) */
#define  RCC_CFGR_SWS_0                      ((uint32_t)0x00000004)        /*!< Bit 0 */
#define  RCC_CFGR_SWS_1                      ((uint32_t)0x00000008)        /*!< Bit 1 */

#define  RCC_CFGR_SWS_HSI                    ((uint32_t)0x00000000)        /*!< HSI oscillator used as system clock */
#define  RCC_CFGR_SWS_HSE                    ((uint32_t)0x00000004)        /*!< HSE oscillator used as system clock */
#define  RCC_CFGR_SWS_PLL                    ((uint32_t)0x00000008)        /*!< PLL used as system clock */

/*!< HPRE configuration */
#define  RCC_CFGR_HPRE                       ((uint32_t)0x000000F0)        /*!< HPRE[3:0] bits (AHB prescaler) */
#define  RCC_CFGR_HPRE_0                     ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  RCC_CFGR_HPRE_1                     ((uint32_t)0x00000020)        /*!< Bit 1 */
#define  RCC_CFGR_HPRE_2                     ((uint32_t)0x00000040)        /*!< Bit 2 */
#define  RCC_CFGR_HPRE_3                     ((uint32_t)0x00000080)        /*!< Bit 3 */

#define  RCC_CFGR_HPRE_DIV1                  ((uint32_t)0x00000000)        /*!< SYSCLK not divided */
#define  RCC_CFGR_HPRE_DIV2                  ((uint32_t)0x00000080)        /*!< SYSCLK divided by 2 */
#define  RCC_CFGR_HPRE_DIV4                  ((uint32_t)0x00000090)        /*!< SYSCLK divided by 4 */
#define  RCC_CFGR_HPRE_DIV8                  ((uint32_t)0x000000A0)        /*!< SYSCLK divided by 8 */
#define  RCC_CFGR_HPRE_DIV16                 ((uint32_t)0x000000B0)        /*!< SYSCLK divided by 16 */
#define  RCC_CFGR_HPRE_DIV64                 ((uint32_t)0x000000C0)        /*!< SYSCLK divided by 64 */
#define  RCC_CFGR_HPRE_DIV128                ((uint32_t)0x000000D0)        /*!< SYSCLK divided by 128 */
#define  RCC_CFGR_HPRE_DIV256                ((uint32_t)0x000000E0)        /*!< SYSCLK divided by 256 */
#define  RCC_CFGR_HPRE_DIV512                ((uint32_t)0x000000F0)        /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define  RCC_CFGR_PPRE1                      ((uint32_t)0x00000700)        /*!< PRE1[2:0] bits (APB1 prescaler) */
#define  RCC_CFGR_PPRE1_0                    ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE1_1                    ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE1_2                    ((uint32_t)0x00000400)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE1_DIV1                 ((uint32_t)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE1_DIV2                 ((uint32_t)0x00000400)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE1_DIV4                 ((uint32_t)0x00000500)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE1_DIV8                 ((uint32_t)0x00000600)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE1_DIV16                ((uint32_t)0x00000700)        /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define  RCC_CFGR_PPRE2                      ((uint32_t)0x00003800)        /*!< PRE2[2:0] bits (APB2 prescaler) */
#define  RCC_CFGR_PPRE2_0                    ((uint32_t)0x00000800)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE2_1                    ((uint32_t)0x00001000)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE2_2                    ((uint32_t)0x00002000)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE2_DIV1                 ((uint32_t)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE2_DIV2                 ((uint32_t)0x00002000)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE2_DIV4                 ((uint32_t)0x00002800)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE2_DIV8                 ((uint32_t)0x00003000)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE2_DIV16                ((uint32_t)0x00003800)        /*!< HCLK divided by 16 */

/*!< ADCPPRE configuration */
#define  RCC_CFGR_ADCPRE                     ((uint32_t)0x0000C000)        /*!< ADCPRE[1:0] bits (ADC prescaler) */
#define  RCC_CFGR_ADCPRE_0                   ((uint32_t)0x00004000)        /*!< Bit 0 */
#define  RCC_CFGR_ADCPRE_1                   ((uint32_t)0x00008000)        /*!< Bit 1 */

#define  RCC_CFGR_ADCPRE_DIV2                ((uint32_t)0x00000000)        /*!< PCLK2 divided by 2 */
#define  RCC_CFGR_ADCPRE_DIV4                ((uint32_t)0x00004000)        /*!< PCLK2 divided by 4 */
#define  RCC_CFGR_ADCPRE_DIV6                ((uint32_t)0x00008000)        /*!< PCLK2 divided by 6 */
#define  RCC_CFGR_ADCPRE_DIV8                ((uint32_t)0x0000C000)        /*!< PCLK2 divided by 8 */

#define  RCC_CFGR_PLLSRC                     ((uint32_t)0x00010000)        /*!< PLL entry clock source */

#define  RCC_CFGR_PLLXTPRE                   ((uint32_t)0x00020000)        /*!< HSE divider for PLL entry */

/*!< PLLMUL configuration */
#define  RCC_CFGR_PLLMULL                    ((uint32_t)0x003C0000)        /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
#define  RCC_CFGR_PLLMULL_0                  ((uint32_t)0x00040000)        /*!< Bit 0 */
#define  RCC_CFGR_PLLMULL_1                  ((uint32_t)0x00080000)        /*!< Bit 1 */
#define  RCC_CFGR_PLLMULL_2                  ((uint32_t)0x00100000)        /*!< Bit 2 */
#define  RCC_CFGR_PLLMULL_3                  ((uint32_t)0x00200000)        /*!< Bit 3 */

#ifdef STM32F10X_CL
 #define  RCC_CFGR_PLLSRC_HSI_Div2           ((uint32_t)0x00000000)        /*!< HSI clock divided by 2 selected as PLL entry clock source */
 #define  RCC_CFGR_PLLSRC_PREDIV1            ((uint32_t)0x00010000)        /*!< PREDIV1 clock selected as PLL entry clock source */

 #define  RCC_CFGR_PLLXTPRE_PREDIV1          ((uint32_t)0x00000000)        /*!< PREDIV1 clock not divided for PLL entry */
 #define  RCC_CFGR_PLLXTPRE_PREDIV1_Div2     ((uint32_t)0x00020000)        /*!< PREDIV1 clock divided by 2 for PLL entry */

 #define  RCC_CFGR_PLLMULL4                  ((uint32_t)0x00080000)        /*!< PLL input clock * 4 */
 #define  RCC_CFGR_PLLMULL5                  ((uint32_t)0x000C0000)        /*!< PLL input clock * 5 */
 #define  RCC_CFGR_PLLMULL6                  ((uint32_t)0x00100000)        /*!< PLL input clock * 6 */
 #define  RCC_CFGR_PLLMULL7                  ((uint32_t)0x00140000)        /*!< PLL input clock * 7 */
 #define  RCC_CFGR_PLLMULL8                  ((uint32_t)0x00180000)        /*!< PLL input clock * 8 */
 #define  RCC_CFGR_PLLMULL9                  ((uint32_t)0x001C0000)        /*!< PLL input clock * 9 */
 #define  RCC_CFGR_PLLMULL6_5                ((uint32_t)0x00340000)        /*!< PLL input clock * 6.5 */
 
 #define  RCC_CFGR_OTGFSPRE                  ((uint32_t)0x00400000)        /*!< USB OTG FS prescaler */
 
/*!< MCO configuration */
 #define  RCC_CFGR_MCO                       ((uint32_t)0x0F000000)        /*!< MCO[3:0] bits (Microcontroller Clock Output) */
 #define  RCC_CFGR_MCO_0                     ((uint32_t)0x01000000)        /*!< Bit 0 */
 #define  RCC_CFGR_MCO_1                     ((uint32_t)0x02000000)        /*!< Bit 1 */
 #define  RCC_CFGR_MCO_2                     ((uint32_t)0x04000000)        /*!< Bit 2 */
 #define  RCC_CFGR_MCO_3                     ((uint32_t)0x08000000)        /*!< Bit 3 */

 #define  RCC_CFGR_MCO_NOCLOCK               ((uint32_t)0x00000000)        /*!< No clock */
 #define  RCC_CFGR_MCO_SYSCLK                ((uint32_t)0x04000000)        /*!< System clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSI                   ((uint32_t)0x05000000)        /*!< HSI clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSE                   ((uint32_t)0x06000000)        /*!< HSE clock selected as MCO source */
 #define  RCC_CFGR_MCO_PLLCLK_Div2           ((uint32_t)0x07000000)        /*!< PLL clock divided by 2 selected as MCO source */
 #define  RCC_CFGR_MCO_PLL2CLK               ((uint32_t)0x08000000)        /*!< PLL2 clock selected as MCO source*/
 #define  RCC_CFGR_MCO_PLL3CLK_Div2          ((uint32_t)0x09000000)        /*!< PLL3 clock divided by 2 selected as MCO source*/
 #define  RCC_CFGR_MCO_Ext_HSE               ((uint32_t)0x0A000000)        /*!< XT1 external 3-25 MHz oscillator clock selected as MCO source */
 #define  RCC_CFGR_MCO_PLL3CLK               ((uint32_t)0x0B000000)        /*!< PLL3 clock selected as MCO source */
#elif defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
 #define  RCC_CFGR_PLLSRC_HSI_Div2           ((uint32_t)0x00000000)        /*!< HSI clock divided by 2 selected as PLL entry clock source */
 #define  RCC_CFGR_PLLSRC_PREDIV1            ((uint32_t)0x00010000)        /*!< PREDIV1 clock selected as PLL entry clock source */

 #define  RCC_CFGR_PLLXTPRE_PREDIV1          ((uint32_t)0x00000000)        /*!< PREDIV1 clock not divided for PLL entry */
 #define  RCC_CFGR_PLLXTPRE_PREDIV1_Div2     ((uint32_t)0x00020000)        /*!< PREDIV1 clock divided by 2 for PLL entry */

 #define  RCC_CFGR_PLLMULL2                  ((uint32_t)0x00000000)        /*!< PLL input clock*2 */
 #define  RCC_CFGR_PLLMULL3                  ((uint32_t)0x00040000)        /*!< PLL input clock*3 */
 #define  RCC_CFGR_PLLMULL4                  ((uint32_t)0x00080000)        /*!< PLL input clock*4 */
 #define  RCC_CFGR_PLLMULL5                  ((uint32_t)0x000C0000)        /*!< PLL input clock*5 */
 #define  RCC_CFGR_PLLMULL6                  ((uint32_t)0x00100000)        /*!< PLL input clock*6 */
 #define  RCC_CFGR_PLLMULL7                  ((uint32_t)0x00140000)        /*!< PLL input clock*7 */
 #define  RCC_CFGR_PLLMULL8                  ((uint32_t)0x00180000)        /*!< PLL input clock*8 */
 #define  RCC_CFGR_PLLMULL9                  ((uint32_t)0x001C0000)        /*!< PLL input clock*9 */
 #define  RCC_CFGR_PLLMULL10                 ((uint32_t)0x00200000)        /*!< PLL input clock10 */
 #define  RCC_CFGR_PLLMULL11                 ((uint32_t)0x00240000)        /*!< PLL input clock*11 */
 #define  RCC_CFGR_PLLMULL12                 ((uint32_t)0x00280000)        /*!< PLL input clock*12 */
 #define  RCC_CFGR_PLLMULL13                 ((uint32_t)0x002C0000)        /*!< PLL input clock*13 */
 #define  RCC_CFGR_PLLMULL14                 ((uint32_t)0x00300000)        /*!< PLL input clock*14 */
 #define  RCC_CFGR_PLLMULL15                 ((uint32_t)0x00340000)        /*!< PLL input clock*15 */
 #define  RCC_CFGR_PLLMULL16                 ((uint32_t)0x00380000)        /*!< PLL input clock*16 */

/*!< MCO configuration */
 #define  RCC_CFGR_MCO                       ((uint32_t)0x07000000)        /*!< MCO[2:0] bits (Microcontroller Clock Output) */
 #define  RCC_CFGR_MCO_0                     ((uint32_t)0x01000000)        /*!< Bit 0 */
 #define  RCC_CFGR_MCO_1                     ((uint32_t)0x02000000)        /*!< Bit 1 */
 #define  RCC_CFGR_MCO_2                     ((uint32_t)0x04000000)        /*!< Bit 2 */

 #define  RCC_CFGR_MCO_NOCLOCK               ((uint32_t)0x00000000)        /*!< No clock */
 #define  RCC_CFGR_MCO_SYSCLK                ((uint32_t)0x04000000)        /*!< System clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSI                   ((uint32_t)0x05000000)        /*!< HSI clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSE                   ((uint32_t)0x06000000)        /*!< HSE clock selected as MCO source  */
 #define  RCC_CFGR_MCO_PLL                   ((uint32_t)0x07000000)        /*!< PLL clock divided by 2 selected as MCO source */
#else
 #define  RCC_CFGR_PLLSRC_HSI_Div2           ((uint32_t)0x00000000)        /*!< HSI clock divided by 2 selected as PLL entry clock source */
 #define  RCC_CFGR_PLLSRC_HSE                ((uint32_t)0x00010000)        /*!< HSE clock selected as PLL entry clock source */

 #define  RCC_CFGR_PLLXTPRE_HSE              ((uint32_t)0x00000000)        /*!< HSE clock not divided for PLL entry */
 #define  RCC_CFGR_PLLXTPRE_HSE_Div2         ((uint32_t)0x00020000)        /*!< HSE clock divided by 2 for PLL entry */

 #define  RCC_CFGR_PLLMULL2                  ((uint32_t)0x00000000)        /*!< PLL input clock*2 */
 #define  RCC_CFGR_PLLMULL3                  ((uint32_t)0x00040000)        /*!< PLL input clock*3 */
 #define  RCC_CFGR_PLLMULL4                  ((uint32_t)0x00080000)        /*!< PLL input clock*4 */
 #define  RCC_CFGR_PLLMULL5                  ((uint32_t)0x000C0000)        /*!< PLL input clock*5 */
 #define  RCC_CFGR_PLLMULL6                  ((uint32_t)0x00100000)        /*!< PLL input clock*6 */
 #define  RCC_CFGR_PLLMULL7                  ((uint32_t)0x00140000)        /*!< PLL input clock*7 */
 #define  RCC_CFGR_PLLMULL8                  ((uint32_t)0x00180000)        /*!< PLL input clock*8 */
 #define  RCC_CFGR_PLLMULL9                  ((uint32_t)0x001C0000)        /*!< PLL input clock*9 */
 #define  RCC_CFGR_PLLMULL10                 ((uint32_t)0x00200000)        /*!< PLL input clock10 */
 #define  RCC_CFGR_PLLMULL11                 ((uint32_t)0x00240000)        /*!< PLL input clock*11 */
 #define  RCC_CFGR_PLLMULL12                 ((uint32_t)0x00280000)        /*!< PLL input clock*12 */
 #define  RCC_CFGR_PLLMULL13                 ((uint32_t)0x002C0000)        /*!< PLL input clock*13 */
 #define  RCC_CFGR_PLLMULL14                 ((uint32_t)0x00300000)        /*!< PLL input clock*14 */
 #define  RCC_CFGR_PLLMULL15                 ((uint32_t)0x00340000)        /*!< PLL input clock*15 */
 #define  RCC_CFGR_PLLMULL16                 ((uint32_t)0x00380000)        /*!< PLL input clock*16 */
 #define  RCC_CFGR_USBPRE                    ((uint32_t)0x00400000)        /*!< USB Device prescaler */

/*!< MCO configuration */
 #define  RCC_CFGR_MCO                       ((uint32_t)0x07000000)        /*!< MCO[2:0] bits (Microcontroller Clock Output) */
 #define  RCC_CFGR_MCO_0                     ((uint32_t)0x01000000)        /*!< Bit 0 */
 #define  RCC_CFGR_MCO_1                     ((uint32_t)0x02000000)        /*!< Bit 1 */
 #define  RCC_CFGR_MCO_2                     ((uint32_t)0x04000000)        /*!< Bit 2 */

 #define  RCC_CFGR_MCO_NOCLOCK               ((uint32_t)0x00000000)        /*!< No clock */
 #define  RCC_CFGR_MCO_SYSCLK                ((uint32_t)0x04000000)        /*!< System clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSI                   ((uint32_t)0x05000000)        /*!< HSI clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSE                   ((uint32_t)0x06000000)        /*!< HSE clock selected as MCO source  */
 #define  RCC_CFGR_MCO_PLL                   ((uint32_t)0x07000000)        /*!< PLL clock divided by 2 selected as MCO source */
#endif /* STM32F10X_CL */

/*!<******************  Bit definition for RCC_CIR register  ********************/
#define  RCC_CIR_LSIRDYF                     ((uint32_t)0x00000001)        /*!< LSI Ready Interrupt flag */
#define  RCC_CIR_LSERDYF                     ((uint32_t)0x00000002)        /*!< LSE Ready Interrupt flag */
#define  RCC_CIR_HSIRDYF                     ((uint32_t)0x00000004)        /*!< HSI Ready Interrupt flag */
#define  RCC_CIR_HSERDYF                     ((uint32_t)0x00000008)        /*!< HSE Ready Interrupt flag */
#define  RCC_CIR_PLLRDYF                     ((uint32_t)0x00000010)        /*!< PLL Ready Interrupt flag */
#define  RCC_CIR_CSSF                        ((uint32_t)0x00000080)        /*!< Clock Security System Interrupt flag */
#define  RCC_CIR_LSIRDYIE                    ((uint32_t)0x00000100)        /*!< LSI Ready Interrupt Enable */
#define  RCC_CIR_LSERDYIE                    ((uint32_t)0x00000200)        /*!< LSE Ready Interrupt Enable */
#define  RCC_CIR_HSIRDYIE                    ((uint32_t)0x00000400)        /*!< HSI Ready Interrupt Enable */
#define  RCC_CIR_HSERDYIE                    ((uint32_t)0x00000800)        /*!< HSE Ready Interrupt Enable */
#define  RCC_CIR_PLLRDYIE                    ((uint32_t)0x00001000)        /*!< PLL Ready Interrupt Enable */
#define  RCC_CIR_LSIRDYC                     ((uint32_t)0x00010000)        /*!< LSI Ready Interrupt Clear */
#define  RCC_CIR_LSERDYC                     ((uint32_t)0x00020000)        /*!< LSE Ready Interrupt Clear */
#define  RCC_CIR_HSIRDYC                     ((uint32_t)0x00040000)        /*!< HSI Ready Interrupt Clear */
#define  RCC_CIR_HSERDYC                     ((uint32_t)0x00080000)        /*!< HSE Ready Interrupt Clear */
#define  RCC_CIR_PLLRDYC                     ((uint32_t)0x00100000)        /*!< PLL Ready Interrupt Clear */
#define  RCC_CIR_CSSC                        ((uint32_t)0x00800000)        /*!< Clock Security System Interrupt Clear */

#ifdef STM32F10X_CL
 #define  RCC_CIR_PLL2RDYF                    ((uint32_t)0x00000020)        /*!< PLL2 Ready Interrupt flag */
 #define  RCC_CIR_PLL3RDYF                    ((uint32_t)0x00000040)        /*!< PLL3 Ready Interrupt flag */
 #define  RCC_CIR_PLL2RDYIE                   ((uint32_t)0x00002000)        /*!< PLL2 Ready Interrupt Enable */
 #define  RCC_CIR_PLL3RDYIE                   ((uint32_t)0x00004000)        /*!< PLL3 Ready Interrupt Enable */
 #define  RCC_CIR_PLL2RDYC                    ((uint32_t)0x00200000)        /*!< PLL2 Ready Interrupt Clear */
 #define  RCC_CIR_PLL3RDYC                    ((uint32_t)0x00400000)        /*!< PLL3 Ready Interrupt Clear */
#endif /* STM32F10X_CL */

/*****************  Bit definition for RCC_APB2RSTR register  *****************/
#define  RCC_APB2RSTR_AFIORST                ((uint32_t)0x00000001)        /*!< Alternate Function I/O reset */
#define  RCC_APB2RSTR_IOPARST                ((uint32_t)0x00000004)        /*!< I/O port A reset */
#define  RCC_APB2RSTR_IOPBRST                ((uint32_t)0x00000008)        /*!< I/O port B reset */
#define  RCC_APB2RSTR_IOPCRST                ((uint32_t)0x00000010)        /*!< I/O port C reset */
#define  RCC_APB2RSTR_IOPDRST                ((uint32_t)0x00000020)        /*!< I/O port D reset */
#define  RCC_APB2RSTR_ADC1RST                ((uint32_t)0x00000200)        /*!< ADC 1 interface reset */

#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && !defined (STM32F10X_HD_VL)
#define  RCC_APB2RSTR_ADC2RST                ((uint32_t)0x00000400)        /*!< ADC 2 interface reset */
#endif

#define  RCC_APB2RSTR_TIM1RST                ((uint32_t)0x00000800)        /*!< TIM1 Timer reset */
#define  RCC_APB2RSTR_SPI1RST                ((uint32_t)0x00001000)        /*!< SPI 1 reset */
#define  RCC_APB2RSTR_USART1RST              ((uint32_t)0x00004000)        /*!< USART1 reset */

#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
#define  RCC_APB2RSTR_TIM15RST               ((uint32_t)0x00010000)        /*!< TIM15 Timer reset */
#define  RCC_APB2RSTR_TIM16RST               ((uint32_t)0x00020000)        /*!< TIM16 Timer reset */
#define  RCC_APB2RSTR_TIM17RST               ((uint32_t)0x00040000)        /*!< TIM17 Timer reset */
#endif

#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
 #define  RCC_APB2RSTR_IOPERST               ((uint32_t)0x00000040)        /*!< I/O port E reset */
#endif /* STM32F10X_LD && STM32F10X_LD_VL */

#if defined (STM32F10X_HD) || defined (STM32F10X_XL)
 #define  RCC_APB2RSTR_IOPFRST               ((uint32_t)0x00000080)        /*!< I/O port F reset */
 #define  RCC_APB2RSTR_IOPGRST               ((uint32_t)0x00000100)        /*!< I/O port G reset */
 #define  RCC_APB2RSTR_TIM8RST               ((uint32_t)0x00002000)        /*!< TIM8 Timer reset */
 #define  RCC_APB2RSTR_ADC3RST               ((uint32_t)0x00008000)        /*!< ADC3 interface reset */
#endif

#if defined (STM32F10X_HD_VL)
 #define  RCC_APB2RSTR_IOPFRST               ((uint32_t)0x00000080)        /*!< I/O port F reset */
 #define  RCC_APB2RSTR_IOPGRST               ((uint32_t)0x00000100)        /*!< I/O port G reset */
#endif

#ifdef STM32F10X_XL
 #define  RCC_APB2RSTR_TIM9RST               ((uint32_t)0x00080000)         /*!< TIM9 Timer reset */
 #define  RCC_APB2RSTR_TIM10RST              ((uint32_t)0x00100000)         /*!< TIM10 Timer reset */
 #define  RCC_APB2RSTR_TIM11RST              ((uint32_t)0x00200000)         /*!< TIM11 Timer reset */
#endif /* STM32F10X_XL */

/*****************  Bit definition for RCC_APB1RSTR register  *****************/
#define  RCC_APB1RSTR_TIM2RST                ((uint32_t)0x00000001)        /*!< Timer 2 reset */
#define  RCC_APB1RSTR_TIM3RST                ((uint32_t)0x00000002)        /*!< Timer 3 reset */
#define  RCC_APB1RSTR_WWDGRST                ((uint32_t)0x00000800)        /*!< Window Watchdog reset */
#define  RCC_APB1RSTR_USART2RST              ((uint32_t)0x00020000)        /*!< USART 2 reset */
#define  RCC_APB1RSTR_I2C1RST                ((uint32_t)0x00200000)        /*!< I2C 1 reset */

#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && !defined (STM32F10X_HD_VL)
#define  RCC_APB1RSTR_CAN1RST                ((uint32_t)0x02000000)        /*!< CAN1 reset */
#endif

#define  RCC_APB1RSTR_BKPRST                 ((uint32_t)0x08000000)        /*!< Backup interface reset */
#define  RCC_APB1RSTR_PWRRST                 ((uint32_t)0x10000000)        /*!< Power interface reset */

#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
 #define  RCC_APB1RSTR_TIM4RST               ((uint32_t)0x00000004)        /*!< Timer 4 reset */
 #define  RCC_APB1RSTR_SPI2RST               ((uint32_t)0x00004000)        /*!< SPI 2 reset */
 #define  RCC_APB1RSTR_USART3RST             ((uint32_t)0x00040000)        /*!< RUSART 3 reset */
 #define  RCC_APB1RSTR_I2C2RST               ((uint32_t)0x00400000)        /*!< I2C 2 reset */
#endif /* STM32F10X_LD && STM32F10X_LD_VL */

#if defined (STM32F10X_HD) || defined (STM32F10X_MD) || defined (STM32F10X_LD) || defined  (STM32F10X_XL)
 #define  RCC_APB1RSTR_USBRST                ((uint32_t)0x00800000)        /*!< USB Device reset */
#endif

#if defined (STM32F10X_HD) || defined  (STM32F10X_CL) || defined  (STM32F10X_XL)
 #define  RCC_APB1RSTR_TIM5RST                ((uint32_t)0x00000008)        /*!< Timer 5 reset */
 #define  RCC_APB1RSTR_TIM6RST                ((uint32_t)0x00000010)        /*!< Timer 6 reset */
 #define  RCC_APB1RSTR_TIM7RST                ((uint32_t)0x00000020)        /*!< Timer 7 reset */
 #define  RCC_APB1RSTR_SPI3RST                ((uint32_t)0x00008000)        /*!< SPI 3 reset */
 #define  RCC_APB1RSTR_UART4RST               ((uint32_t)0x00080000)        /*!< UART 4 reset */
 #define  RCC_APB1RSTR_UART5RST               ((uint32_t)0x00100000)        /*!< UART 5 reset */
 #define  RCC_APB1RSTR_DACRST                 ((uint32_t)0x20000000)        /*!< DAC interface reset */
#endif

#if defined (STM32F10X_LD_VL) || defined  (STM32F10X_MD_VL) || defined  (STM32F10X_HD_VL)
 #define  RCC_APB1RSTR_TIM6RST                ((uint32_t)0x00000010)        /*!< Timer 6 reset */
 #define  RCC_APB1RSTR_TIM7RST                ((uint32_t)0x00000020)        /*!< Timer 7 reset */
 #define  RCC_APB1RSTR_DACRST                 ((uint32_t)0x20000000)        /*!< DAC interface reset */
 #define  RCC_APB1RSTR_CECRST                 ((uint32_t)0x40000000)        /*!< CEC interface reset */ 
#endif

#if defined  (STM32F10X_HD_VL)
 #define  RCC_APB1RSTR_TIM5RST                ((uint32_t)0x00000008)        /*!< Timer 5 reset */
 #define  RCC_APB1RSTR_TIM12RST               ((uint32_t)0x00000040)        /*!< TIM12 Timer reset */
 #define  RCC_APB1RSTR_TIM13RST               ((uint32_t)0x00000080)        /*!< TIM13 Timer reset */
 #define  RCC_APB1RSTR_TIM14RST               ((uint32_t)0x00000100)        /*!< TIM14 Timer reset */
 #define  RCC_APB1RSTR_SPI3RST                ((uint32_t)0x00008000)        /*!< SPI 3 reset */ 
 #define  RCC_APB1RSTR_UART4RST               ((uint32_t)0x00080000)        /*!< UART 4 reset */
 #define  RCC_APB1RSTR_UART5RST               ((uint32_t)0x00100000)        /*!< UART 5 reset */ 
#endif

#ifdef STM32F10X_CL
 #define  RCC_APB1RSTR_CAN2RST                ((uint32_t)0x04000000)        /*!< CAN2 reset */
#endif /* STM32F10X_CL */

#ifdef STM32F10X_XL
 #define  RCC_APB1RSTR_TIM12RST               ((uint32_t)0x00000040)         /*!< TIM12 Timer reset */
 #define  RCC_APB1RSTR_TIM13RST               ((uint32_t)0x00000080)         /*!< TIM13 Timer reset */
 #define  RCC_APB1RSTR_TIM14RST               ((uint32_t)0x00000100)         /*!< TIM14 Timer reset */
#endif /* STM32F10X_XL */

/******************  Bit definition for RCC_AHBENR register  ******************/
#define  RCC_AHBENR_DMA1EN                   ((uint16_t)0x0001)            /*!< DMA1 clock enable */
#define  RCC_AHBENR_SRAMEN                   ((uint16_t)0x0004)            /*!< SRAM interface clock enable */
#define  RCC_AHBENR_FLITFEN                  ((uint16_t)0x0010)            /*!< FLITF clock enable */
#define  RCC_AHBENR_CRCEN                    ((uint16_t)0x0040)            /*!< CRC clock enable */

#if defined (STM32F10X_HD) || defined  (STM32F10X_CL) || defined  (STM32F10X_HD_VL)
 #define  RCC_AHBENR_DMA2EN                  ((uint16_t)0x0002)            /*!< DMA2 clock enable */
#endif

#if defined (STM32F10X_HD) || defined (STM32F10X_XL)
 #define  RCC_AHBENR_FSMCEN                  ((uint16_t)0x0100)            /*!< FSMC clock enable */
 #define  RCC_AHBENR_SDIOEN                  ((uint16_t)0x0400)            /*!< SDIO clock enable */
#endif

#if defined (STM32F10X_HD_VL)
 #define  RCC_AHBENR_FSMCEN                  ((uint16_t)0x0100)            /*!< FSMC clock enable */
#endif

#ifdef STM32F10X_CL
 #define  RCC_AHBENR_OTGFSEN                 ((uint32_t)0x00001000)         /*!< USB OTG FS clock enable */
 #define  RCC_AHBENR_ETHMACEN                ((uint32_t)0x00004000)         /*!< ETHERNET MAC clock enable */
 #define  RCC_AHBENR_ETHMACTXEN              ((uint32_t)0x00008000)         /*!< ETHERNET MAC Tx clock enable */
 #define  RCC_AHBENR_ETHMACRXEN              ((uint32_t)0x00010000)         /*!< ETHERNET MAC Rx clock enable */
#endif /* STM32F10X_CL */

/******************  Bit definition for RCC_APB2ENR register  *****************/
#define  RCC_APB2ENR_AFIOEN                  ((uint32_t)0x00000001)         /*!< Alternate Function I/O clock enable */
#define  RCC_APB2ENR_IOPAEN                  ((uint32_t)0x00000004)         /*!< I/O port A clock enable */
#define  RCC_APB2ENR_IOPBEN                  ((uint32_t)0x00000008)         /*!< I/O port B clock enable */
#define  RCC_APB2ENR_IOPCEN                  ((uint32_t)0x00000010)         /*!< I/O port C clock enable */
#define  RCC_APB2ENR_IOPDEN                  ((uint32_t)0x00000020)         /*!< I/O port D clock enable */
#define  RCC_APB2ENR_ADC1EN                  ((uint32_t)0x00000200)         /*!< ADC 1 interface clock enable */

#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && !defined (STM32F10X_HD_VL)
#define  RCC_APB2ENR_ADC2EN                  ((uint32_t)0x00000400)         /*!< ADC 2 interface clock enable */
#endif

#define  RCC_APB2ENR_TIM1EN                  ((uint32_t)0x00000800)         /*!< TIM1 Timer clock enable */
#define  RCC_APB2ENR_SPI1EN                  ((uint32_t)0x00001000)         /*!< SPI 1 clock enable */
#define  RCC_APB2ENR_USART1EN                ((uint32_t)0x00004000)         /*!< USART1 clock enable */

#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
#define  RCC_APB2ENR_TIM15EN                 ((uint32_t)0x00010000)         /*!< TIM15 Timer clock enable */
#define  RCC_APB2ENR_TIM16EN                 ((uint32_t)0x00020000)         /*!< TIM16 Timer clock enable */
#define  RCC_APB2ENR_TIM17EN                 ((uint32_t)0x00040000)         /*!< TIM17 Timer clock enable */
#endif

#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
 #define  RCC_APB2ENR_IOPEEN                 ((uint32_t)0x00000040)         /*!< I/O port E clock enable */
#endif /* STM32F10X_LD && STM32F10X_LD_VL */

#if defined (STM32F10X_HD) || defined (STM32F10X_XL)
 #define  RCC_APB2ENR_IOPFEN                 ((uint32_t)0x00000080)         /*!< I/O port F clock enable */
 #define  RCC_APB2ENR_IOPGEN                 ((uint32_t)0x00000100)         /*!< I/O port G clock enable */
 #define  RCC_APB2ENR_TIM8EN                 ((uint32_t)0x00002000)         /*!< TIM8 Timer clock enable */
 #define  RCC_APB2ENR_ADC3EN                 ((uint32_t)0x00008000)         /*!< DMA1 clock enable */
#endif

#if defined (STM32F10X_HD_VL)
 #define  RCC_APB2ENR_IOPFEN                 ((uint32_t)0x00000080)         /*!< I/O port F clock enable */
 #define  RCC_APB2ENR_IOPGEN                 ((uint32_t)0x00000100)         /*!< I/O port G clock enable */
#endif

#ifdef STM32F10X_XL
 #define  RCC_APB2ENR_TIM9EN                 ((uint32_t)0x00080000)         /*!< TIM9 Timer clock enable  */
 #define  RCC_APB2ENR_TIM10EN                ((uint32_t)0x00100000)         /*!< TIM10 Timer clock enable  */
 #define  RCC_APB2ENR_TIM11EN                ((uint32_t)0x00200000)         /*!< TIM11 Timer clock enable */
#endif

/*****************  Bit definition for RCC_APB1ENR register  ******************/
#define  RCC_APB1ENR_TIM2EN                  ((uint32_t)0x00000001)        /*!< Timer 2 clock enabled*/
#define  RCC_APB1ENR_TIM3EN                  ((uint32_t)0x00000002)        /*!< Timer 3 clock enable */
#define  RCC_APB1ENR_WWDGEN                  ((uint32_t)0x00000800)        /*!< Window Watchdog clock enable */
#define  RCC_APB1ENR_USART2EN                ((uint32_t)0x00020000)        /*!< USART 2 clock enable */
#define  RCC_APB1ENR_I2C1EN                  ((uint32_t)0x00200000)        /*!< I2C 1 clock enable */

#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && !defined (STM32F10X_HD_VL)
#define  RCC_APB1ENR_CAN1EN                  ((uint32_t)0x02000000)        /*!< CAN1 clock enable */
#endif

#define  RCC_APB1ENR_BKPEN                   ((uint32_t)0x08000000)        /*!< Backup interface clock enable */
#define  RCC_APB1ENR_PWREN                   ((uint32_t)0x10000000)        /*!< Power interface clock enable */

#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
 #define  RCC_APB1ENR_TIM4EN                 ((uint32_t)0x00000004)        /*!< Timer 4 clock enable */
 #define  RCC_APB1ENR_SPI2EN                 ((uint32_t)0x00004000)        /*!< SPI 2 clock enable */
 #define  RCC_APB1ENR_USART3EN               ((uint32_t)0x00040000)        /*!< USART 3 clock enable */
 #define  RCC_APB1ENR_I2C2EN                 ((uint32_t)0x00400000)        /*!< I2C 2 clock enable */
#endif /* STM32F10X_LD && STM32F10X_LD_VL */

#if defined (STM32F10X_HD) || defined (STM32F10X_MD) || defined  (STM32F10X_LD)
 #define  RCC_APB1ENR_USBEN                  ((uint32_t)0x00800000)        /*!< USB Device clock enable */
#endif

#if defined (STM32F10X_HD) || defined  (STM32F10X_CL)
 #define  RCC_APB1ENR_TIM5EN                 ((uint32_t)0x00000008)        /*!< Timer 5 clock enable */
 #define  RCC_APB1ENR_TIM6EN                 ((uint32_t)0x00000010)        /*!< Timer 6 clock enable */
 #define  RCC_APB1ENR_TIM7EN                 ((uint32_t)0x00000020)        /*!< Timer 7 clock enable */
 #define  RCC_APB1ENR_SPI3EN                 ((uint32_t)0x00008000)        /*!< SPI 3 clock enable */
 #define  RCC_APB1ENR_UART4EN                ((uint32_t)0x00080000)        /*!< UART 4 clock enable */
 #define  RCC_APB1ENR_UART5EN                ((uint32_t)0x00100000)        /*!< UART 5 clock enable */
 #define  RCC_APB1ENR_DACEN                  ((uint32_t)0x20000000)        /*!< DAC interface clock enable */
#endif

#if defined (STM32F10X_LD_VL) || defined  (STM32F10X_MD_VL) || defined  (STM32F10X_HD_VL)
 #define  RCC_APB1ENR_TIM6EN                 ((uint32_t)0x00000010)        /*!< Timer 6 clock enable */
 #define  RCC_APB1ENR_TIM7EN                 ((uint32_t)0x00000020)        /*!< Timer 7 clock enable */
 #define  RCC_APB1ENR_DACEN                  ((uint32_t)0x20000000)        /*!< DAC interface clock enable */
 #define  RCC_APB1ENR_CECEN                  ((uint32_t)0x40000000)        /*!< CEC interface clock enable */ 
#endif

#ifdef STM32F10X_HD_VL
 #define  RCC_APB1ENR_TIM5EN                 ((uint32_t)0x00000008)        /*!< Timer 5 clock enable */
 #define  RCC_APB1ENR_TIM12EN                ((uint32_t)0x00000040)         /*!< TIM12 Timer clock enable  */
 #define  RCC_APB1ENR_TIM13EN                ((uint32_t)0x00000080)         /*!< TIM13 Timer clock enable  */
 #define  RCC_APB1ENR_TIM14EN                ((uint32_t)0x00000100)         /*!< TIM14 Timer clock enable */
 #define  RCC_APB1ENR_SPI3EN                 ((uint32_t)0x00008000)        /*!< SPI 3 clock enable */
 #define  RCC_APB1ENR_UART4EN                ((uint32_t)0x00080000)        /*!< UART 4 clock enable */
 #define  RCC_APB1ENR_UART5EN                ((uint32_t)0x00100000)        /*!< UART 5 clock enable */ 
#endif /* STM32F10X_HD_VL */

#ifdef STM32F10X_CL
 #define  RCC_APB1ENR_CAN2EN                  ((uint32_t)0x04000000)        /*!< CAN2 clock enable */
#endif /* STM32F10X_CL */

#ifdef STM32F10X_XL
 #define  RCC_APB1ENR_TIM12EN                ((uint32_t)0x00000040)         /*!< TIM12 Timer clock enable  */
 #define  RCC_APB1ENR_TIM13EN                ((uint32_t)0x00000080)         /*!< TIM13 Timer clock enable  */
 #define  RCC_APB1ENR_TIM14EN                ((uint32_t)0x00000100)         /*!< TIM14 Timer clock enable */
#endif /* STM32F10X_XL */

/*******************  Bit definition for RCC_BDCR register  *******************/
#define  RCC_BDCR_LSEON                      ((uint32_t)0x00000001)        /*!< External Low Speed oscillator enable */
#define  RCC_BDCR_LSERDY                     ((uint32_t)0x00000002)        /*!< External Low Speed oscillator Ready */
#define  RCC_BDCR_LSEBYP                     ((uint32_t)0x00000004)        /*!< External Low Speed oscillator Bypass */

#define  RCC_BDCR_RTCSEL                     ((uint32_t)0x00000300)        /*!< RTCSEL[1:0] bits (RTC clock source selection) */
#define  RCC_BDCR_RTCSEL_0                   ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  RCC_BDCR_RTCSEL_1                   ((uint32_t)0x00000200)        /*!< Bit 1 */

/*!< RTC congiguration */
#define  RCC_BDCR_RTCSEL_NOCLOCK             ((uint32_t)0x00000000)        /*!< No clock */
#define  RCC_BDCR_RTCSEL_LSE                 ((uint32_t)0x00000100)        /*!< LSE oscillator clock used as RTC clock */
#define  RCC_BDCR_RTCSEL_LSI                 ((uint32_t)0x00000200)        /*!< LSI oscillator clock used as RTC clock */
#define  RCC_BDCR_RTCSEL_HSE                 ((uint32_t)0x00000300)        /*!< HSE oscillator clock divided by 128 used as RTC clock */

#define  RCC_BDCR_RTCEN                      ((uint32_t)0x00008000)        /*!< RTC clock enable */
#define  RCC_BDCR_BDRST                      ((uint32_t)0x00010000)        /*!< Backup domain software reset  */

/*******************  Bit definition for RCC_CSR register  ********************/  
#define  RCC_CSR_LSION                       ((uint32_t)0x00000001)        /*!< Internal Low Speed oscillator enable */
#define  RCC_CSR_LSIRDY                      ((uint32_t)0x00000002)        /*!< Internal Low Speed oscillator Ready */
#define  RCC_CSR_RMVF                        ((uint32_t)0x01000000)        /*!< Remove reset flag */
#define  RCC_CSR_PINRSTF                     ((uint32_t)0x04000000)        /*!< PIN reset flag */
#define  RCC_CSR_PORRSTF                     ((uint32_t)0x08000000)        /*!< POR/PDR reset flag */
#define  RCC_CSR_SFTRSTF                     ((uint32_t)0x10000000)        /*!< Software Reset flag */
#define  RCC_CSR_IWDGRSTF                    ((uint32_t)0x20000000)        /*!< Independent Watchdog reset flag */
#define  RCC_CSR_WWDGRSTF                    ((uint32_t)0x40000000)        /*!< Window watchdog reset flag */
#define  RCC_CSR_LPWRRSTF                    ((uint32_t)0x80000000)        /*!< Low-Power reset flag */

#ifdef STM32F10X_CL
/*******************  Bit definition for RCC_AHBRSTR register  ****************/
 #define  RCC_AHBRSTR_OTGFSRST               ((uint32_t)0x00001000)         /*!< USB OTG FS reset */
 #define  RCC_AHBRSTR_ETHMACRST              ((uint32_t)0x00004000)         /*!< ETHERNET MAC reset */

/*******************  Bit definition for RCC_CFGR2 register  ******************/
/*!< PREDIV1 configuration */
 #define  RCC_CFGR2_PREDIV1                  ((uint32_t)0x0000000F)        /*!< PREDIV1[3:0] bits */
 #define  RCC_CFGR2_PREDIV1_0                ((uint32_t)0x00000001)        /*!< Bit 0 */
 #define  RCC_CFGR2_PREDIV1_1                ((uint32_t)0x00000002)        /*!< Bit 1 */
 #define  RCC_CFGR2_PREDIV1_2                ((uint32_t)0x00000004)        /*!< Bit 2 */
 #define  RCC_CFGR2_PREDIV1_3                ((uint32_t)0x00000008)        /*!< Bit 3 */

 #define  RCC_CFGR2_PREDIV1_DIV1             ((uint32_t)0x00000000)        /*!< PREDIV1 input clock not divided */
 #define  RCC_CFGR2_PREDIV1_DIV2             ((uint32_t)0x00000001)        /*!< PREDIV1 input clock divided by 2 */
 #define  RCC_CFGR2_PREDIV1_DIV3             ((uint32_t)0x00000002)        /*!< PREDIV1 input clock divided by 3 */
 #define  RCC_CFGR2_PREDIV1_DIV4             ((uint32_t)0x00000003)        /*!< PREDIV1 input clock divided by 4 */
 #define  RCC_CFGR2_PREDIV1_DIV5             ((uint32_t)0x00000004)        /*!< PREDIV1 input clock divided by 5 */
 #define  RCC_CFGR2_PREDIV1_DIV6             ((uint32_t)0x00000005)        /*!< PREDIV1 input clock divided by 6 */
 #define  RCC_CFGR2_PREDIV1_DIV7             ((uint32_t)0x00000006)        /*!< PREDIV1 input clock divided by 7 */
 #define  RCC_CFGR2_PREDIV1_DIV8             ((uint32_t)0x00000007)        /*!< PREDIV1 input clock divided by 8 */
 #define  RCC_CFGR2_PREDIV1_DIV9             ((uint32_t)0x00000008)        /*!< PREDIV1 input clock divided by 9 */
 #define  RCC_CFGR2_PREDIV1_DIV10            ((uint32_t)0x00000009)        /*!< PREDIV1 input clock divided by 10 */
 #define  RCC_CFGR2_PREDIV1_DIV11            ((uint32_t)0x0000000A)        /*!< PREDIV1 input clock divided by 11 */
 #define  RCC_CFGR2_PREDIV1_DIV12            ((uint32_t)0x0000000B)        /*!< PREDIV1 input clock divided by 12 */
 #define  RCC_CFGR2_PREDIV1_DIV13            ((uint32_t)0x0000000C)        /*!< PREDIV1 input clock divided by 13 */
 #define  RCC_CFGR2_PREDIV1_DIV14            ((uint32_t)0x0000000D)        /*!< PREDIV1 input clock divided by 14 */
 #define  RCC_CFGR2_PREDIV1_DIV15            ((uint32_t)0x0000000E)        /*!< PREDIV1 input clock divided by 15 */
 #define  RCC_CFGR2_PREDIV1_DIV16            ((uint32_t)0x0000000F)        /*!< PREDIV1 input clock divided by 16 */

/*!< PREDIV2 configuration */
 #define  RCC_CFGR2_PREDIV2                  ((uint32_t)0x000000F0)        /*!< PREDIV2[3:0] bits */
 #define  RCC_CFGR2_PREDIV2_0                ((uint32_t)0x00000010)        /*!< Bit 0 */
 #define  RCC_CFGR2_PREDIV2_1                ((uint32_t)0x00000020)        /*!< Bit 1 */
 #define  RCC_CFGR2_PREDIV2_2                ((uint32_t)0x00000040)        /*!< Bit 2 */
 #define  RCC_CFGR2_PREDIV2_3                ((uint32_t)0x00000080)        /*!< Bit 3 */

 #define  RCC_CFGR2_PREDIV2_DIV1             ((uint32_t)0x00000000)        /*!< PREDIV2 input clock not divided */
 #define  RCC_CFGR2_PREDIV2_DIV2             ((uint32_t)0x00000010)        /*!< PREDIV2 input clock divided by 2 */
 #define  RCC_CFGR2_PREDIV2_DIV3             ((uint32_t)0x00000020)        /*!< PREDIV2 input clock divided by 3 */
 #define  RCC_CFGR2_PREDIV2_DIV4             ((uint32_t)0x00000030)        /*!< PREDIV2 input clock divided by 4 */
 #define  RCC_CFGR2_PREDIV2_DIV5             ((uint32_t)0x00000040)        /*!< PREDIV2 input clock divided by 5 */
 #define  RCC_CFGR2_PREDIV2_DIV6             ((uint32_t)0x00000050)        /*!< PREDIV2 input clock divided by 6 */
 #define  RCC_CFGR2_PREDIV2_DIV7             ((uint32_t)0x00000060)        /*!< PREDIV2 input clock divided by 7 */
 #define  RCC_CFGR2_PREDIV2_DIV8             ((uint32_t)0x00000070)        /*!< PREDIV2 input clock divided by 8 */
 #define  RCC_CFGR2_PREDIV2_DIV9             ((uint32_t)0x00000080)        /*!< PREDIV2 input clock divided by 9 */
 #define  RCC_CFGR2_PREDIV2_DIV10            ((uint32_t)0x00000090)        /*!< PREDIV2 input clock divided by 10 */
 #define  RCC_CFGR2_PREDIV2_DIV11            ((uint32_t)0x000000A0)        /*!< PREDIV2 input clock divided by 11 */
 #define  RCC_CFGR2_PREDIV2_DIV12            ((uint32_t)0x000000B0)        /*!< PREDIV2 input clock divided by 12 */
 #define  RCC_CFGR2_PREDIV2_DIV13            ((uint32_t)0x000000C0)        /*!< PREDIV2 input clock divided by 13 */
 #define  RCC_CFGR2_PREDIV2_DIV14            ((uint32_t)0x000000D0)        /*!< PREDIV2 input clock divided by 14 */
 #define  RCC_CFGR2_PREDIV2_DIV15            ((uint32_t)0x000000E0)        /*!< PREDIV2 input clock divided by 15 */
 #define  RCC_CFGR2_PREDIV2_DIV16            ((uint32_t)0x000000F0)        /*!< PREDIV2 input clock divided by 16 */

/*!< PLL2MUL configuration */
 #define  RCC_CFGR2_PLL2MUL                  ((uint32_t)0x00000F00)        /*!< PLL2MUL[3:0] bits */
 #define  RCC_CFGR2_PLL2MUL_0                ((uint32_t)0x00000100)        /*!< Bit 0 */
 #define  RCC_CFGR2_PLL2MUL_1                ((uint32_t)0x00000200)        /*!< Bit 1 */
 #define  RCC_CFGR2_PLL2MUL_2                ((uint32_t)0x00000400)        /*!< Bit 2 */
 #define  RCC_CFGR2_PLL2MUL_3                ((uint32_t)0x00000800)        /*!< Bit 3 */

 #define  RCC_CFGR2_PLL2MUL8                 ((uint32_t)0x00000600)        /*!< PLL2 input clock * 8 */
 #define  RCC_CFGR2_PLL2MUL9                 ((uint32_t)0x00000700)        /*!< PLL2 input clock * 9 */
 #define  RCC_CFGR2_PLL2MUL10                ((uint32_t)0x00000800)        /*!< PLL2 input clock * 10 */
 #define  RCC_CFGR2_PLL2MUL11                ((uint32_t)0x00000900)        /*!< PLL2 input clock * 11 */
 #define  RCC_CFGR2_PLL2MUL12                ((uint32_t)0x00000A00)        /*!< PLL2 input clock * 12 */
 #define  RCC_CFGR2_PLL2MUL13                ((uint32_t)0x00000B00)        /*!< PLL2 input clock * 13 */
 #define  RCC_CFGR2_PLL2MUL14                ((uint32_t)0x00000C00)        /*!< PLL2 input clock * 14 */
 #define  RCC_CFGR2_PLL2MUL16                ((uint32_t)0x00000E00)        /*!< PLL2 input clock * 16 */
 #define  RCC_CFGR2_PLL2MUL20                ((uint32_t)0x00000F00)        /*!< PLL2 input clock * 20 */

/*!< PLL3MUL configuration */
 #define  RCC_CFGR2_PLL3MUL                  ((uint32_t)0x0000F000)        /*!< PLL3MUL[3:0] bits */
 #define  RCC_CFGR2_PLL3MUL_0                ((uint32_t)0x00001000)        /*!< Bit 0 */
 #define  RCC_CFGR2_PLL3MUL_1                ((uint32_t)0x00002000)        /*!< Bit 1 */
 #define  RCC_CFGR2_PLL3MUL_2                ((uint32_t)0x00004000)        /*!< Bit 2 */
 #define  RCC_CFGR2_PLL3MUL_3                ((uint32_t)0x00008000)        /*!< Bit 3 */

 #define  RCC_CFGR2_PLL3MUL8                 ((uint32_t)0x00006000)        /*!< PLL3 input clock * 8 */
 #define  RCC_CFGR2_PLL3MUL9                 ((uint32_t)0x00007000)        /*!< PLL3 input clock * 9 */
 #define  RCC_CFGR2_PLL3MUL10                ((uint32_t)0x00008000)        /*!< PLL3 input clock * 10 */
 #define  RCC_CFGR2_PLL3MUL11                ((uint32_t)0x00009000)        /*!< PLL3 input clock * 11 */
 #define  RCC_CFGR2_PLL3MUL12                ((uint32_t)0x0000A000)        /*!< PLL3 input clock * 12 */
 #define  RCC_CFGR2_PLL3MUL13                ((uint32_t)0x0000B000)        /*!< PLL3 input clock * 13 */
 #define  RCC_CFGR2_PLL3MUL14                ((uint32_t)0x0000C000)        /*!< PLL3 input clock * 14 */
 #define  RCC_CFGR2_PLL3MUL16                ((uint32_t)0x0000E000)        /*!< PLL3 input clock * 16 */
 #define  RCC_CFGR2_PLL3MUL20                ((uint32_t)0x0000F000)        /*!< PLL3 input clock * 20 */

 #define  RCC_CFGR2_PREDIV1SRC               ((uint32_t)0x00010000)        /*!< PREDIV1 entry clock source */
 #define  RCC_CFGR2_PREDIV1SRC_PLL2          ((uint32_t)0x00010000)        /*!< PLL2 selected as PREDIV1 entry clock source */
 #define  RCC_CFGR2_PREDIV1SRC_HSE           ((uint32_t)0x00000000)        /*!< HSE selected as PREDIV1 entry clock source */
 #define  RCC_CFGR2_I2S2SRC                  ((uint32_t)0x00020000)        /*!< I2S2 entry clock source */
 #define  RCC_CFGR2_I2S3SRC                  ((uint32_t)0x00040000)        /*!< I2S3 clock source */
#endif /* STM32F10X_CL */

#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
/*******************  Bit definition for RCC_CFGR2 register  ******************/
/*!< PREDIV1 configuration */
 #define  RCC_CFGR2_PREDIV1                  ((uint32_t)0x0000000F)        /*!< PREDIV1[3:0] bits */
 #define  RCC_CFGR2_PREDIV1_0                ((uint32_t)0x00000001)        /*!< Bit 0 */
 #define  RCC_CFGR2_PREDIV1_1                ((uint32_t)0x00000002)        /*!< Bit 1 */
 #define  RCC_CFGR2_PREDIV1_2                ((uint32_t)0x00000004)        /*!< Bit 2 */
 #define  RCC_CFGR2_PREDIV1_3                ((uint32_t)0x00000008)        /*!< Bit 3 */

 #define  RCC_CFGR2_PREDIV1_DIV1             ((uint32_t)0x00000000)        /*!< PREDIV1 input clock not divided */
 #define  RCC_CFGR2_PREDIV1_DIV2             ((uint32_t)0x00000001)        /*!< PREDIV1 input clock divided by 2 */
 #define  RCC_CFGR2_PREDIV1_DIV3             ((uint32_t)0x00000002)        /*!< PREDIV1 input clock divided by 3 */
 #define  RCC_CFGR2_PREDIV1_DIV4             ((uint32_t)0x00000003)        /*!< PREDIV1 input clock divided by 4 */
 #define  RCC_CFGR2_PREDIV1_DIV5             ((uint32_t)0x00000004)        /*!< PREDIV1 input clock divided by 5 */
 #define  RCC_CFGR2_PREDIV1_DIV6             ((uint32_t)0x00000005)        /*!< PREDIV1 input clock divided by 6 */
 #define  RCC_CFGR2_PREDIV1_DIV7             ((uint32_t)0x00000006)        /*!< PREDIV1 input clock divided by 7 */
 #define  RCC_CFGR2_PREDIV1_DIV8             ((uint32_t)0x00000007)        /*!< PREDIV1 input clock divided by 8 */
 #define  RCC_CFGR2_PREDIV1_DIV9             ((uint32_t)0x00000008)        /*!< PREDIV1 input clock divided by 9 */
 #define  RCC_CFGR2_PREDIV1_DIV10            ((uint32_t)0x00000009)        /*!< PREDIV1 input clock divided by 10 */
 #define  RCC_CFGR2_PREDIV1_DIV11            ((uint32_t)0x0000000A)        /*!< PREDIV1 input clock divided by 11 */
 #define  RCC_CFGR2_PREDIV1_DIV12            ((uint32_t)0x0000000B)        /*!< PREDIV1 input clock divided by 12 */
 #define  RCC_CFGR2_PREDIV1_DIV13            ((uint32_t)0x0000000C)        /*!< PREDIV1 input clock divided by 13 */
 #define  RCC_CFGR2_PREDIV1_DIV14            ((uint32_t)0x0000000D)        /*!< PREDIV1 input clock divided by 14 */
 #define  RCC_CFGR2_PREDIV1_DIV15            ((uint32_t)0x0000000E)        /*!< PREDIV1 input clock divided by 15 */
 #define  RCC_CFGR2_PREDIV1_DIV16            ((uint32_t)0x0000000F)        /*!< PREDIV1 input clock divided by 16 */
#endif



/******************************************************************************/
/*                                                                            */
/*                General Purpose and Alternate Function I/O                  */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for GPIO_CRL register  *******************/
#define  GPIO_CRL_MODE                       ((uint32_t)0x33333333)        /*!< Port x mode bits */

#define  GPIO_CRL_MODE0                      ((uint32_t)0x00000003)        /*!< MODE0[1:0] bits (Port x mode bits, pin 0) */
#define  GPIO_CRL_MODE0_0                    ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  GPIO_CRL_MODE0_1                    ((uint32_t)0x00000002)        /*!< Bit 1 */

#define  GPIO_CRL_MODE1                      ((uint32_t)0x00000030)        /*!< MODE1[1:0] bits (Port x mode bits, pin 1) */
#define  GPIO_CRL_MODE1_0                    ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  GPIO_CRL_MODE1_1                    ((uint32_t)0x00000020)        /*!< Bit 1 */

#define  GPIO_CRL_MODE2                      ((uint32_t)0x00000300)        /*!< MODE2[1:0] bits (Port x mode bits, pin 2) */
#define  GPIO_CRL_MODE2_0                    ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  GPIO_CRL_MODE2_1                    ((uint32_t)0x00000200)        /*!< Bit 1 */

#define  GPIO_CRL_MODE3                      ((uint32_t)0x00003000)        /*!< MODE3[1:0] bits (Port x mode bits, pin 3) */
#define  GPIO_CRL_MODE3_0                    ((uint32_t)0x00001000)        /*!< Bit 0 */
#define  GPIO_CRL_MODE3_1                    ((uint32_t)0x00002000)        /*!< Bit 1 */

#define  GPIO_CRL_MODE4                      ((uint32_t)0x00030000)        /*!< MODE4[1:0] bits (Port x mode bits, pin 4) */
#define  GPIO_CRL_MODE4_0                    ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  GPIO_CRL_MODE4_1                    ((uint32_t)0x00020000)        /*!< Bit 1 */

#define  GPIO_CRL_MODE5                      ((uint32_t)0x00300000)        /*!< MODE5[1:0] bits (Port x mode bits, pin 5) */
#define  GPIO_CRL_MODE5_0                    ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  GPIO_CRL_MODE5_1                    ((uint32_t)0x00200000)        /*!< Bit 1 */

#define  GPIO_CRL_MODE6                      ((uint32_t)0x03000000)        /*!< MODE6[1:0] bits (Port x mode bits, pin 6) */
#define  GPIO_CRL_MODE6_0                    ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  GPIO_CRL_MODE6_1                    ((uint32_t)0x02000000)        /*!< Bit 1 */

#define  GPIO_CRL_MODE7                      ((uint32_t)0x30000000)        /*!< MODE7[1:0] bits (Port x mode bits, pin 7) */
#define  GPIO_CRL_MODE7_0                    ((uint32_t)0x10000000)        /*!< Bit 0 */
#define  GPIO_CRL_MODE7_1                    ((uint32_t)0x20000000)        /*!< Bit 1 */

#define  GPIO_CRL_CNF                        ((uint32_t)0xCCCCCCCC)        /*!< Port x configuration bits */

#define  GPIO_CRL_CNF0                       ((uint32_t)0x0000000C)        /*!< CNF0[1:0] bits (Port x configuration bits, pin 0) */
#define  GPIO_CRL_CNF0_0                     ((uint32_t)0x00000004)        /*!< Bit 0 */
#define  GPIO_CRL_CNF0_1                     ((uint32_t)0x00000008)        /*!< Bit 1 */

#define  GPIO_CRL_CNF1                       ((uint32_t)0x000000C0)        /*!< CNF1[1:0] bits (Port x configuration bits, pin 1) */
#define  GPIO_CRL_CNF1_0                     ((uint32_t)0x00000040)        /*!< Bit 0 */
#define  GPIO_CRL_CNF1_1                     ((uint32_t)0x00000080)        /*!< Bit 1 */

#define  GPIO_CRL_CNF2                       ((uint32_t)0x00000C00)        /*!< CNF2[1:0] bits (Port x configuration bits, pin 2) */
#define  GPIO_CRL_CNF2_0                     ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  GPIO_CRL_CNF2_1                     ((uint32_t)0x00000800)        /*!< Bit 1 */

#define  GPIO_CRL_CNF3                       ((uint32_t)0x0000C000)        /*!< CNF3[1:0] bits (Port x configuration bits, pin 3) */
#define  GPIO_CRL_CNF3_0                     ((uint32_t)0x00004000)        /*!< Bit 0 */
#define  GPIO_CRL_CNF3_1                     ((uint32_t)0x00008000)        /*!< Bit 1 */

#define  GPIO_CRL_CNF4                       ((uint32_t)0x000C0000)        /*!< CNF4[1:0] bits (Port x configuration bits, pin 4) */
#define  GPIO_CRL_CNF4_0                     ((uint32_t)0x00040000)        /*!< Bit 0 */
#define  GPIO_CRL_CNF4_1                     ((uint32_t)0x00080000)        /*!< Bit 1 */

#define  GPIO_CRL_CNF5                       ((uint32_t)0x00C00000)        /*!< CNF5[1:0] bits (Port x configuration bits, pin 5) */
#define  GPIO_CRL_CNF5_0                     ((uint32_t)0x00400000)        /*!< Bit 0 */
#define  GPIO_CRL_CNF5_1                     ((uint32_t)0x00800000)        /*!< Bit 1 */

#define  GPIO_CRL_CNF6                       ((uint32_t)0x0C000000)        /*!< CNF6[1:0] bits (Port x configuration bits, pin 6) */
#define  GPIO_CRL_CNF6_0                     ((uint32_t)0x04000000)        /*!< Bit 0 */
#define  GPIO_CRL_CNF6_1                     ((uint32_t)0x08000000)        /*!< Bit 1 */

#define  GPIO_CRL_CNF7                       ((uint32_t)0xC0000000)        /*!< CNF7[1:0] bits (Port x configuration bits, pin 7) */
#define  GPIO_CRL_CNF7_0                     ((uint32_t)0x40000000)        /*!< Bit 0 */
#define  GPIO_CRL_CNF7_1                     ((uint32_t)0x80000000)        /*!< Bit 1 */

/*******************  Bit definition for GPIO_CRH register  *******************/
#define  GPIO_CRH_MODE                       ((uint32_t)0x33333333)        /*!< Port x mode bits */

#define  GPIO_CRH_MODE8                      ((uint32_t)0x00000003)        /*!< MODE8[1:0] bits (Port x mode bits, pin 8) */
#define  GPIO_CRH_MODE8_0                    ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  GPIO_CRH_MODE8_1                    ((uint32_t)0x00000002)        /*!< Bit 1 */

#define  GPIO_CRH_MODE9                      ((uint32_t)0x00000030)        /*!< MODE9[1:0] bits (Port x mode bits, pin 9) */
#define  GPIO_CRH_MODE9_0                    ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  GPIO_CRH_MODE9_1                    ((uint32_t)0x00000020)        /*!< Bit 1 */

#define  GPIO_CRH_MODE10                     ((uint32_t)0x00000300)        /*!< MODE10[1:0] bits (Port x mode bits, pin 10) */
#define  GPIO_CRH_MODE10_0                   ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  GPIO_CRH_MODE10_1                   ((uint32_t)0x00000200)        /*!< Bit 1 */

#define  GPIO_CRH_MODE11                     ((uint32_t)0x00003000)        /*!< MODE11[1:0] bits (Port x mode bits, pin 11) */
#define  GPIO_CRH_MODE11_0                   ((uint32_t)0x00001000)        /*!< Bit 0 */
#define  GPIO_CRH_MODE11_1                   ((uint32_t)0x00002000)        /*!< Bit 1 */

#define  GPIO_CRH_MODE12                     ((uint32_t)0x00030000)        /*!< MODE12[1:0] bits (Port x mode bits, pin 12) */
#define  GPIO_CRH_MODE12_0                   ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  GPIO_CRH_MODE12_1                   ((uint32_t)0x00020000)        /*!< Bit 1 */

#define  GPIO_CRH_MODE13                     ((uint32_t)0x00300000)        /*!< MODE13[1:0] bits (Port x mode bits, pin 13) */
#define  GPIO_CRH_MODE13_0                   ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  GPIO_CRH_MODE13_1                   ((uint32_t)0x00200000)        /*!< Bit 1 */

#define  GPIO_CRH_MODE14                     ((uint32_t)0x03000000)        /*!< MODE14[1:0] bits (Port x mode bits, pin 14) */
#define  GPIO_CRH_MODE14_0                   ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  GPIO_CRH_MODE14_1                   ((uint32_t)0x02000000)        /*!< Bit 1 */

#define  GPIO_CRH_MODE15                     ((uint32_t)0x30000000)        /*!< MODE15[1:0] bits (Port x mode bits, pin 15) */
#define  GPIO_CRH_MODE15_0                   ((uint32_t)0x10000000)        /*!< Bit 0 */
#define  GPIO_CRH_MODE15_1                   ((uint32_t)0x20000000)        /*!< Bit 1 */

#define  GPIO_CRH_CNF                        ((uint32_t)0xCCCCCCCC)        /*!< Port x configuration bits */

#define  GPIO_CRH_CNF8                       ((uint32_t)0x0000000C)        /*!< CNF8[1:0] bits (Port x configuration bits, pin 8) */
#define  GPIO_CRH_CNF8_0                     ((uint32_t)0x00000004)        /*!< Bit 0 */
#define  GPIO_CRH_CNF8_1                     ((uint32_t)0x00000008)        /*!< Bit 1 */

#define  GPIO_CRH_CNF9                       ((uint32_t)0x000000C0)        /*!< CNF9[1:0] bits (Port x configuration bits, pin 9) */
#define  GPIO_CRH_CNF9_0                     ((uint32_t)0x00000040)        /*!< Bit 0 */
#define  GPIO_CRH_CNF9_1                     ((uint32_t)0x00000080)        /*!< Bit 1 */

#define  GPIO_CRH_CNF10                      ((uint32_t)0x00000C00)        /*!< CNF10[1:0] bits (Port x configuration bits, pin 10) */
#define  GPIO_CRH_CNF10_0                    ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  GPIO_CRH_CNF10_1                    ((uint32_t)0x00000800)        /*!< Bit 1 */

#define  GPIO_CRH_CNF11                      ((uint32_t)0x0000C000)        /*!< CNF11[1:0] bits (Port x configuration bits, pin 11) */
#define  GPIO_CRH_CNF11_0                    ((uint32_t)0x00004000)        /*!< Bit 0 */
#define  GPIO_CRH_CNF11_1                    ((uint32_t)0x00008000)        /*!< Bit 1 */

#define  GPIO_CRH_CNF12                      ((uint32_t)0x000C0000)        /*!< CNF12[1:0] bits (Port x configuration bits, pin 12) */
#define  GPIO_CRH_CNF12_0                    ((uint32_t)0x00040000)        /*!< Bit 0 */
#define  GPIO_CRH_CNF12_1                    ((uint32_t)0x00080000)        /*!< Bit 1 */

#define  GPIO_CRH_CNF13                      ((uint32_t)0x00C00000)        /*!< CNF13[1:0] bits (Port x configuration bits, pin 13) */
#define  GPIO_CRH_CNF13_0                    ((uint32_t)0x00400000)        /*!< Bit 0 */
#define  GPIO_CRH_CNF13_1                    ((uint32_t)0x00800000)        /*!< Bit 1 */

#define  GPIO_CRH_CNF14                      ((uint32_t)0x0C000000)        /*!< CNF14[1:0] bits (Port x configuration bits, pin 14) */
#define  GPIO_CRH_CNF14_0                    ((uint32_t)0x04000000)        /*!< Bit 0 */
#define  GPIO_CRH_CNF14_1                    ((uint32_t)0x08000000)        /*!< Bit 1 */

#define  GPIO_CRH_CNF15                      ((uint32_t)0xC0000000)        /*!< CNF15[1:0] bits (Port x configuration bits, pin 15) */
#define  GPIO_CRH_CNF15_0                    ((uint32_t)0x40000000)        /*!< Bit 0 */
#define  GPIO_CRH_CNF15_1                    ((uint32_t)0x80000000)        /*!< Bit 1 */

/*!<******************  Bit definition for GPIO_IDR register  *******************/
#define GPIO_IDR_IDR0                        ((uint16_t)0x0001)            /*!< Port input data, bit 0 */
#define GPIO_IDR_IDR1                        ((uint16_t)0x0002)            /*!< Port input data, bit 1 */
#define GPIO_IDR_IDR2                        ((uint16_t)0x0004)            /*!< Port input data, bit 2 */
#define GPIO_IDR_IDR3                        ((uint16_t)0x0008)            /*!< Port input data, bit 3 */
#define GPIO_IDR_IDR4                        ((uint16_t)0x0010)            /*!< Port input data, bit 4 */
#define GPIO_IDR_IDR5                        ((uint16_t)0x0020)            /*!< Port input data, bit 5 */
#define GPIO_IDR_IDR6                        ((uint16_t)0x0040)            /*!< Port input data, bit 6 */
#define GPIO_IDR_IDR7                        ((uint16_t)0x0080)            /*!< Port input data, bit 7 */
#define GPIO_IDR_IDR8                        ((uint16_t)0x0100)            /*!< Port input data, bit 8 */
#define GPIO_IDR_IDR9                        ((uint16_t)0x0200)            /*!< Port input data, bit 9 */
#define GPIO_IDR_IDR10                       ((uint16_t)0x0400)            /*!< Port input data, bit 10 */
#define GPIO_IDR_IDR11                       ((uint16_t)0x0800)            /*!< Port input data, bit 11 */
#define GPIO_IDR_IDR12                       ((uint16_t)0x1000)            /*!< Port input data, bit 12 */
#define GPIO_IDR_IDR13                       ((uint16_t)0x2000)            /*!< Port input data, bit 13 */
#define GPIO_IDR_IDR14                       ((uint16_t)0x4000)            /*!< Port input data, bit 14 */
#define GPIO_IDR_IDR15                       ((uint16_t)0x8000)            /*!< Port input data, bit 15 */

/*******************  Bit definition for GPIO_ODR register  *******************/
#define GPIO_ODR_ODR0                        ((uint16_t)0x0001)            /*!< Port output data, bit 0 */
#define GPIO_ODR_ODR1                        ((uint16_t)0x0002)            /*!< Port output data, bit 1 */
#define GPIO_ODR_ODR2                        ((uint16_t)0x0004)            /*!< Port output data, bit 2 */
#define GPIO_ODR_ODR3                        ((uint16_t)0x0008)            /*!< Port output data, bit 3 */
#define GPIO_ODR_ODR4                        ((uint16_t)0x0010)            /*!< Port output data, bit 4 */
#define GPIO_ODR_ODR5                        ((uint16_t)0x0020)            /*!< Port output data, bit 5 */
#define GPIO_ODR_ODR6                        ((uint16_t)0x0040)            /*!< Port output data, bit 6 */
#define GPIO_ODR_ODR7                        ((uint16_t)0x0080)            /*!< Port output data, bit 7 */
#define GPIO_ODR_ODR8                        ((uint16_t)0x0100)            /*!< Port output data, bit 8 */
#define GPIO_ODR_ODR9                        ((uint16_t)0x0200)            /*!< Port output data, bit 9 */
#define GPIO_ODR_ODR10                       ((uint16_t)0x0400)            /*!< Port output data, bit 10 */
#define GPIO_ODR_ODR11                       ((uint16_t)0x0800)            /*!< Port output data, bit 11 */
#define GPIO_ODR_ODR12                       ((uint16_t)0x1000)            /*!< Port output data, bit 12 */
#define GPIO_ODR_ODR13                       ((uint16_t)0x2000)            /*!< Port output data, bit 13 */
#define GPIO_ODR_ODR14                       ((uint16_t)0x4000)            /*!< Port output data, bit 14 */
#define GPIO_ODR_ODR15                       ((uint16_t)0x8000)            /*!< Port output data, bit 15 */

/******************  Bit definition for GPIO_BSRR register  *******************/
#define GPIO_BSRR_BS0                        ((uint32_t)0x00000001)        /*!< Port x Set bit 0 */
#define GPIO_BSRR_BS1                        ((uint32_t)0x00000002)        /*!< Port x Set bit 1 */
#define GPIO_BSRR_BS2                        ((uint32_t)0x00000004)        /*!< Port x Set bit 2 */
#define GPIO_BSRR_BS3                        ((uint32_t)0x00000008)        /*!< Port x Set bit 3 */
#define GPIO_BSRR_BS4                        ((uint32_t)0x00000010)        /*!< Port x Set bit 4 */
#define GPIO_BSRR_BS5                        ((uint32_t)0x00000020)        /*!< Port x Set bit 5 */
#define GPIO_BSRR_BS6                        ((uint32_t)0x00000040)        /*!< Port x Set bit 6 */
#define GPIO_BSRR_BS7                        ((uint32_t)0x00000080)        /*!< Port x Set bit 7 */
#define GPIO_BSRR_BS8                        ((uint32_t)0x00000100)        /*!< Port x Set bit 8 */
#define GPIO_BSRR_BS9                        ((uint32_t)0x00000200)        /*!< Port x Set bit 9 */
#define GPIO_BSRR_BS10                       ((uint32_t)0x00000400)        /*!< Port x Set bit 10 */
#define GPIO_BSRR_BS11                       ((uint32_t)0x00000800)        /*!< Port x Set bit 11 */
#define GPIO_BSRR_BS12                       ((uint32_t)0x00001000)        /*!< Port x Set bit 12 */
#define GPIO_BSRR_BS13                       ((uint32_t)0x00002000)        /*!< Port x Set bit 13 */
#define GPIO_BSRR_BS14                       ((uint32_t)0x00004000)        /*!< Port x Set bit 14 */
#define GPIO_BSRR_BS15                       ((uint32_t)0x00008000)        /*!< Port x Set bit 15 */

#define GPIO_BSRR_BR0                        ((uint32_t)0x00010000)        /*!< Port x Reset bit 0 */
#define GPIO_BSRR_BR1                        ((uint32_t)0x00020000)        /*!< Port x Reset bit 1 */
#define GPIO_BSRR_BR2                        ((uint32_t)0x00040000)        /*!< Port x Reset bit 2 */
#define GPIO_BSRR_BR3                        ((uint32_t)0x00080000)        /*!< Port x Reset bit 3 */
#define GPIO_BSRR_BR4                        ((uint32_t)0x00100000)        /*!< Port x Reset bit 4 */
#define GPIO_BSRR_BR5                        ((uint32_t)0x00200000)        /*!< Port x Reset bit 5 */
#define GPIO_BSRR_BR6                        ((uint32_t)0x00400000)        /*!< Port x Reset bit 6 */
#define GPIO_BSRR_BR7                        ((uint32_t)0x00800000)        /*!< Port x Reset bit 7 */
#define GPIO_BSRR_BR8                        ((uint32_t)0x01000000)        /*!< Port x Reset bit 8 */
#define GPIO_BSRR_BR9                        ((uint32_t)0x02000000)        /*!< Port x Reset bit 9 */
#define GPIO_BSRR_BR10                       ((uint32_t)0x04000000)        /*!< Port x Reset bit 10 */
#define GPIO_BSRR_BR11                       ((uint32_t)0x08000000)        /*!< Port x Reset bit 11 */
#define GPIO_BSRR_BR12                       ((uint32_t)0x10000000)        /*!< Port x Reset bit 12 */
#define GPIO_BSRR_BR13                       ((uint32_t)0x20000000)        /*!< Port x Reset bit 13 */
#define GPIO_BSRR_BR14                       ((uint32_t)0x40000000)        /*!< Port x Reset bit 14 */
#define GPIO_BSRR_BR15                       ((uint32_t)0x80000000)        /*!< Port x Reset bit 15 */

/*******************  Bit definition for GPIO_BRR register  *******************/
#define GPIO_BRR_BR0                         ((uint16_t)0x0001)            /*!< Port x Reset bit 0 */
#define GPIO_BRR_BR1                         ((uint16_t)0x0002)            /*!< Port x Reset bit 1 */
#define GPIO_BRR_BR2                         ((uint16_t)0x0004)            /*!< Port x Reset bit 2 */
#define GPIO_BRR_BR3                         ((uint16_t)0x0008)            /*!< Port x Reset bit 3 */
#define GPIO_BRR_BR4                         ((uint16_t)0x0010)            /*!< Port x Reset bit 4 */
#define GPIO_BRR_BR5                         ((uint16_t)0x0020)            /*!< Port x Reset bit 5 */
#define GPIO_BRR_BR6                         ((uint16_t)0x0040)            /*!< Port x Reset bit 6 */
#define GPIO_BRR_BR7                         ((uint16_t)0x0080)            /*!< Port x Reset bit 7 */
#define GPIO_BRR_BR8                         ((uint16_t)0x0100)            /*!< Port x Reset bit 8 */
#define GPIO_BRR_BR9                         ((uint16_t)0x0200)            /*!< Port x Reset bit 9 */
#define GPIO_BRR_BR10                        ((uint16_t)0x0400)            /*!< Port x Reset bit 10 */
#define GPIO_BRR_BR11                        ((uint16_t)0x0800)            /*!< Port x Reset bit 11 */
#define GPIO_BRR_BR12                        ((uint16_t)0x1000)            /*!< Port x Reset bit 12 */
#define GPIO_BRR_BR13                        ((uint16_t)0x2000)            /*!< Port x Reset bit 13 */
#define GPIO_BRR_BR14                        ((uint16_t)0x4000)            /*!< Port x Reset bit 14 */
#define GPIO_BRR_BR15                        ((uint16_t)0x8000)            /*!< Port x Reset bit 15 */

/******************  Bit definition for GPIO_LCKR register  *******************/
#define GPIO_LCKR_LCK0                       ((uint32_t)0x00000001)        /*!< Port x Lock bit 0 */
#define GPIO_LCKR_LCK1                       ((uint32_t)0x00000002)        /*!< Port x Lock bit 1 */
#define GPIO_LCKR_LCK2                       ((uint32_t)0x00000004)        /*!< Port x Lock bit 2 */
#define GPIO_LCKR_LCK3                       ((uint32_t)0x00000008)        /*!< Port x Lock bit 3 */
#define GPIO_LCKR_LCK4                       ((uint32_t)0x00000010)        /*!< Port x Lock bit 4 */
#define GPIO_LCKR_LCK5                       ((uint32_t)0x00000020)        /*!< Port x Lock bit 5 */
#define GPIO_LCKR_LCK6                       ((uint32_t)0x00000040)        /*!< Port x Lock bit 6 */
#define GPIO_LCKR_LCK7                       ((uint32_t)0x00000080)        /*!< Port x Lock bit 7 */
#define GPIO_LCKR_LCK8                       ((uint32_t)0x00000100)        /*!< Port x Lock bit 8 */
#define GPIO_LCKR_LCK9                       ((uint32_t)0x00000200)        /*!< Port x Lock bit 9 */
#define GPIO_LCKR_LCK10                      ((uint32_t)0x00000400)        /*!< Port x Lock bit 10 */
#define GPIO_LCKR_LCK11                      ((uint32_t)0x00000800)        /*!< Port x Lock bit 11 */
#define GPIO_LCKR_LCK12                      ((uint32_t)0x00001000)        /*!< Port x Lock bit 12 */
#define GPIO_LCKR_LCK13                      ((uint32_t)0x00002000)        /*!< Port x Lock bit 13 */
#define GPIO_LCKR_LCK14                      ((uint32_t)0x00004000)        /*!< Port x Lock bit 14 */
#define GPIO_LCKR_LCK15                      ((uint32_t)0x00008000)        /*!< Port x Lock bit 15 */
#define GPIO_LCKR_LCKK                       ((uint32_t)0x00010000)        /*!< Lock key */




#endif
