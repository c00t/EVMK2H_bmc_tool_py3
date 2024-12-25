/******************************************************************************
 *
 * File	Name:       bmc_map.h
 *
 * Description: This file contains the Pinout for the Stellaris on the K2K evm.
 * 
 ******************************************************************************/
/****************************************************************************
 * Copyright (c) 2012 Texas Instruments Incorporated - http://www.ti.com
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef BMC_MAP_H_
#define BMC_MAP_H_

/* =================== Includes ===================== */
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
/* ====================== Defines ===================== */
/* Flash Register Defines */
#define BOARD_TYPE_REG          FLASH_USERREG0          // Flash register that contains the board type
#define BOARD_TYPE_ADDR         0x80000000              // Address of flash register USER_REG0
#define BOARD_VER_REG           FLASH_USERREG1          // Flash register that contains the board version
#define BOARD_VER_ADDR          0x80000001              // Address of flash register USER_REG1
#define BOARD_SN_REG            FLASH_USERREG2          // Flash register that contains the board serial number
#define BOARD_SN_ADDR           0x80000002              // Address of flash register USER_REG2
#define BOOTCFG_ADDR            0x75100000              // Address of flash register Boot CFG

#define BOOTCFG_PORT            FLASH_BOOTCFG_PORT_G    // GPIO Port for the Boot CFG pin
#define BOOTCFG_PIN             FLASH_BOOTCFG_PIN_7     // Pin for the Boot CFG
#define BOOTCFG_POL             FLASH_BOOTCFG_POL       // Polarity of the Boot CFG pin

/* Boot Mode Defines */
#define BOOT_LOCATION           0x7FC00                 // Location in flash to start programming bootmodes
#define CHECK_OFFSET            0x200                   // Offset from bootmode location to create the check byte

/* SPI Device Key Defines */
#define CLOCK_GEN_1             0x00                    // SPI 0 CS 0
#define CLOCK_GEN_2             0x01                    // SPI 0 CS 1
#define CLOCK_GEN_3             0x02                    // SPI 0 CS 2
#define GPIO_EXPANDER_A         0x03                    // SPI 0 CS 3
#define GPIO_EXPANDER_B         0x04                    // SPI 0 CS 4
#define GPIO_EXPANDER_C         0x05                    // SPI 0 CS 5
#define GPIO_EXPANDER_D         0x06                    // SPI 0 CS 6
#define LCD_SPI                 0x10                    // SPI 1 CS 0

/* I2C Device Key Defines */
#define SOC_I2C                 0x00                    // SOC I2C Key
#define EEPROM50                0x01                    // EEPROM Lower Range Key
#define EEPROM51                0x02                    // EEPROM Upper Range Key
#define UCD_9090                0x03                    // PM Bus/UCD9090 Key
#define UCD_9244_0              0x04                    // PM Bus/UCD9244_0 Key
#define UCD_9244_1              0x05                    // PM Bus/UCD9244_1 Key

/* GPIO Port Mask Defines */
#define GPIO_PORT_A             0x00000001              // GPIO Port A
#define GPIO_PORT_B             0x00000002              // GPIO Port B
#define GPIO_PORT_C             0x00000004              // GPIO Port C
#define GPIO_PORT_D             0x00000008              // GPIO Port D
#define GPIO_PORT_E             0x00000010              // GPIO Port E
#define GPIO_PORT_F             0x00000020              // GPIO Port F
#define GPIO_PORT_G             0x00000040              // GPIO Port G
#define GPIO_PORT_H             0x00000080              // GPIO Port H
#define GPIO_PORT_J             0x00000100              // GPIO Port J

#define GPIO_PORT_XA            0x00010000              // GPIO Expander Port A
#define GPIO_PORT_XB            0x00020000              // GPIO Expander Port B
#define GPIO_PORT_XC            0x00040000              // GPIO Expander Port C
#define GPIO_PORT_XD            0x00080000              // GPIO Expander Port D

/* Pin Definitions */
#define PIN_0                   0x0001
#define PIN_1                   0x0002
#define PIN_2                   0x0004
#define PIN_3                   0x0008
#define PIN_4                   0x0010
#define PIN_5                   0x0020
#define PIN_6                   0x0040
#define PIN_7                   0x0080
#define PIN_8                   0x0100
#define PIN_9                   0x0200
#define PIN_A                   0x0400
#define PIN_B                   0x0800
#define PIN_C                   0x1000
#define PIN_D                   0x2000
#define PIN_E                   0x4000
#define PIN_F                   0x8000
#define ALL_PINS                0xFFFF

/* GPIO Port Abstractions */
/* Port A */
#define U0_RX_PORT              GPIO_PORT_A             // UART0 Receive        PA0
#define U0_TX_PORT              GPIO_PORT_A             // UART0 Transmit       PA1
#define SPI0_CLK_PORT           GPIO_PORT_A             // SPI0 CLK             PA2
#define SPI0_CS0_PORT           GPIO_PORT_A             // SPI0 CS0             PA3
#define SPI0_MISO_PORT          GPIO_PORT_A             // SPI0 MISO            PA4
#define SPI0_MOSI_PORT          GPIO_PORT_A             // SPI0 MOSI            PA5
#define I2C1_SCL_PORT           GPIO_PORT_A             // I2C1 CLOCK           PA6
#define I2C1_SDA_PORT           GPIO_PORT_A             // I2C1 DATA            PA7
/* Port B */
#define U1_RX_PORT              GPIO_PORT_B             // UART1 Receive        PB0
#define U1_TX_PORT              GPIO_PORT_B             // UART1 Transmit       PB1
#define I2C0_SCL_PORT           GPIO_PORT_B             // I2C0 CLOCK           PB2
#define I2C0_SDA_PORT           GPIO_PORT_B             // I2C0 DATA            PB3
#define MMC_GA0_PORT            GPIO_PORT_B             // MMC GA0              PB4
#define MMC_GA1_PORT            GPIO_PORT_B             // MMC GA1              PB5
#define MMC_GA2_PORT            GPIO_PORT_B             // MMC GA2              PB6
#define MMC_GAPU_PORT           GPIO_PORT_B             // MMC GAPU             PB7
/* Port C */
#define JTAG_TCK_PORT           GPIO_PORT_C             // JTAG                 PC0
#define JTAG_TMS_PORT           GPIO_PORT_C             // JTAG                 PC1
#define JTAG_TDI_PORT           GPIO_PORT_C             // JTAG                 PC2
#define JTAG_TDO_PORT           GPIO_PORT_C             // JTAG                 PC3
#define SPI0_CS1_PORT           GPIO_PORT_C             // SPI0 CS1             PC4
#define SPI0_CS2_PORT           GPIO_PORT_C             // SPI0 CS2             PC5
#define SPI0_CS3_PORT           GPIO_PORT_C             // SPI0 CS3             PC6
#define SPI0_CS4_PORT           GPIO_PORT_C             // SPI0 CS4             PC7
/* Port D */
#define MAIN_POWER_START_PORT   GPIO_PORT_D             //                      PD0
#define MAIN_POWER_GOOD_PORT    GPIO_PORT_D             //                      PD1
#define SOC_POWER_START_PORT    GPIO_PORT_D             //                      PD2
#define SOC_POWER_GOOD_PORT     GPIO_PORT_D             //                      PD3
//#define NC                    GPIO_PORT_D             // NC                   PD4
//#define NC                    GPIO_PORT_D             // NC                   PD5
#define VCC3V3_MP_ALT_DET_PORT  GPIO_PORT_D             //                      PD6
#define LCD_RST_PORT            GPIO_PORT_D             // LCD Reset            PD7
/* Port E */
#define SPI1_CLK_PORT           GPIO_PORT_E             // SPI1 CLK             PE0
#define SPI1_CS0_PORT           GPIO_PORT_E             // SPI1 CS0             PE1
#define SPI1_MISO_PORT          GPIO_PORT_E             // SPI1 MISO            PE2
#define SPI1_MOSI_PORT          GPIO_PORT_E             // SPI1 MOSI            PE3
#define SOC_I2C_EN_PORT         GPIO_PORT_E             // SOC I2C Enable       PE4
#define PW_SEQ_I2C_EN_PORT      GPIO_PORT_E             // PW Seq I2C Enable    PE5
#define PW_SEQ_RST_PORT         GPIO_PORT_E             // PW Seq Reset         PE6
#define PHY_RST_PORT            GPIO_PORT_E             // PHY Reset            PE7
/* Port F */
//#define NC                    GPIO_PORT_F             // NC                   PF0
#define LCD_A0_PORT             GPIO_PORT_F             // LCD A0               PF1
#define SPI0_CS5_PORT           GPIO_PORT_F             // SPI0 CS5             PF2
#define SPI0_CS6_PORT           GPIO_PORT_F             // SPI0 CS6             PF3
#define SPI0_GPIO_INT0_PORT     GPIO_PORT_F             // SPI0 GPIO/INT0       PF4
#define SPI0_GPIO_INT1_PORT     GPIO_PORT_F             // SPI0 GPIO/INT1       PF5
#define SPI0_GPIO_INT2_PORT     GPIO_PORT_F             // SPI0 GPIO/INT2       PF6
#define SPI0_GPIO_INT3_PORT     GPIO_PORT_F             // SPI0 GPIO/INT3       PF7
/* Port G */
#define MMC_ENABLE_N_PORT       GPIO_PORT_G             //                      PG0
#define NOR_WP_PORT             GPIO_PORT_G             //                      PG1
#define PCIECLK_MCU_PD_PORT     GPIO_PORT_G             //                      PG2
#define PCIECLK_OE_PORT         GPIO_PORT_G             //                      PG3
#define PMBUS_CTL_PORT          GPIO_PORT_G             //                      PG4
#define PMBUS_ALER_PORT         GPIO_PORT_G             //                      PG5
#define SPI_GPIO_RESET_PORT     GPIO_PORT_G             // SPI GPIO Reset       PG6
#define MCU_BOOTSELECT_PORT     GPIO_PORT_G             // MCU bootselect       PG7
/* Port H */
#define DIP_SW_B0_PORT          GPIO_PORT_H             // DIP Switch 0         PH0
#define DIP_SW_B1_PORT          GPIO_PORT_H             // DIP Switch 1         PH1
#define DIP_SW_B2_PORT          GPIO_PORT_H             // DIP Switch 2         PH2
#define DIP_SW_B3_PORT          GPIO_PORT_H             // DIP Switch 3         PH3
#define PCIECLK_MUX_SEL_PORT    GPIO_PORT_H             //                      PH4
#define MMC_PS_N0_PORT          GPIO_PORT_H             //                      PH5
#define MMC_RED_LED_PORT        GPIO_PORT_H             //                      PH6
#define MMC_BLUE_LED_PORT       GPIO_PORT_H             //                      PH7
/* Port J */
#define WARM_RESET_PORT         GPIO_PORT_J             // Warm Reset           PJ0
#define FULL_RESET_PORT         GPIO_PORT_J             // Full Reset           PJ1
#define TRGRSTZ_PORT            GPIO_PORT_J             //                      PJ2
//#define NC                    GPIO_PORT_J             // SPI0 CS6             PJ3
//#define NC                    GPIO_PORT_J             // SPI0 GPIO/INT0       PJ4
//#define NC                    GPIO_PORT_J             // SPI0 GPIO/INT1       PJ5
//#define NC                    GPIO_PORT_J             // SPI0 GPIO/INT2       PJ6
//#define NC                    GPIO_PORT_J             // SPI0 GPIO/INT3       PJ7

/* Port XA */
#define SOC_GPIO_00_PORT        GPIO_PORT_XA            // SOC GPIO 0           A0
#define SOC_GPIO_01_PORT        GPIO_PORT_XA            // SOC GPIO 1           A1
#define SOC_GPIO_02_PORT        GPIO_PORT_XA            // SOC GPIO 2           A2
#define SOC_GPIO_03_PORT        GPIO_PORT_XA            // SOC GPIO 3           A3
#define SOC_GPIO_04_PORT        GPIO_PORT_XA            // SOC GPIO 4           A4
#define SOC_GPIO_05_PORT        GPIO_PORT_XA            // SOC GPIO 5           A5
#define SOC_GPIO_06_PORT        GPIO_PORT_XA            // SOC GPIO 6           A6
#define SOC_GPIO_07_PORT        GPIO_PORT_XA            // SOC GPIO 7           A7
#define SOC_GPIO_08_PORT        GPIO_PORT_XA            // SOC GPIO 8           B0
#define SOC_GPIO_09_PORT        GPIO_PORT_XA            // SOC GPIO 9           B1
#define SOC_GPIO_10_PORT        GPIO_PORT_XA            // SOC GPIO 10          B2
#define SOC_GPIO_11_PORT        GPIO_PORT_XA            // SOC GPIO 11          B3
#define SOC_GPIO_12_PORT        GPIO_PORT_XA            // SOC GPIO 12          B4
#define SOC_GPIO_13_PORT        GPIO_PORT_XA            // SOC GPIO 13          B5
#define SOC_GPIO_14_PORT        GPIO_PORT_XA            // SOC GPIO 14          B6
#define SOC_GPIO_15_PORT        GPIO_PORT_XA            // SOC GPIO 15          B7
/* Port XB */
#define EMIF_OEZ_PORT           GPIO_PORT_XB            // No Connect           A0
#define EMIF_DIR_PORT           GPIO_PORT_XB            // No Connect           A1
#define MCU_GPS_ENABLE_PORT     GPIO_PORT_XB            // MCU_GPS_ENABLE       A2
#define SOC_VPPB_EN_PORT        GPIO_PORT_XB            // SOC_VPPB_EN          A3
#define SOC_GPIO_16_PORT        GPIO_PORT_XB            // SOC GPIO 16          A4
#define PLLLOCK_LED_PORT        GPIO_PORT_XB            // PLLLOCK LED          A5
#define SOC_PLLLOCK_PORT        GPIO_PORT_XB            // SOC PLLLOCK          A6
//#define NC                    GPIO_PORT_XB            // No Connect           A7
#define NAND_WPz_PORT           GPIO_PORT_XB            // NAND WP              B0
#define EEPROM_WP_PORT          GPIO_PORT_XB            // EEPROM WP            B1
#define BD_PRESENT_PORT         GPIO_PORT_XB            // BD_PRESENT           B2
#define BD_ID0_PORT             GPIO_PORT_XB            // BD_ID0               B3
#define BD_ID1_PORT             GPIO_PORT_XB            // BD_ID1               B4
#define BD_ID2_PORT             GPIO_PORT_XB            // BD_ID2               B5
#define SOC_SCL1_PORT           GPIO_PORT_XB            // SOC SCL              B6
#define SOC_SDA1_PORT           GPIO_PORT_XB            // SOC SDA              B7
/* Port XC */
#define SOC_CORECLKSEL_PORT     GPIO_PORT_XC            // SOC_CORECLKSEL       A0
#define SOC_PACLKSEL_PORT       GPIO_PORT_XC            // SOC_PACLKSEL         A1
#define SOC_HOUT_PORT           GPIO_PORT_XC            // SOC_HOUT             A2
#define SOC_NMIZ_PORT           GPIO_PORT_XC            // SOC_NMIZ             A3
#define SOC_LRESETZ_PORT        GPIO_PORT_XC            // SOC_LRESETZ          A4
#define SOC_LRESETNMIENZ_PORT   GPIO_PORT_XC            // SOC_LRESETNMIENZ     A5
#define SOC_BOOTCOMPLETE_PORT   GPIO_PORT_XC            // SOC_BOOTCOMPLETE     A6
#define SOC_RESETSTATZ_PORT     GPIO_PORT_XC            // SOC_RESETSTATZ       A7
#define SOC_CORESEL0_PORT       GPIO_PORT_XC            // SOC_CORESEL0         B0
#define SOC_CORESEL1_PORT       GPIO_PORT_XC            // SOC_CORESEL1         B1
#define SOC_CORESEL2_PORT       GPIO_PORT_XC            // SOC_CORESEL2         B2
#define SOC_CORESEL3_PORT       GPIO_PORT_XC            // SOC_CORESEL3         B3
#define TIMI_MUX_OEz_PORT       GPIO_PORT_XC            // TIMI_MUX_OEz         B4
#define SOC_RESETFULLZ_PORT     GPIO_PORT_XC            // SOC_RESETFULLZ       B5
#define SOC_RESETZ_PORT         GPIO_PORT_XC            // SOC_RESETZ           B6
#define SOC_PORZ_PORT           GPIO_PORT_XC            // SOC_PORZ             B7
/* Port XD */
#define PHY_INT_PORT            GPIO_PORT_XD            // PHY_INT              A0
#define MCU_RESETSTATz_PORT     GPIO_PORT_XD            // MCU_RESETSTATz       A1
#define EXT_PS_PORT             GPIO_PORT_XD            // EXT_PS               A2
#define MCU_EMU_DET_PORT        GPIO_PORT_XD            // MCU_EMU_DET          A3
#define CLK3_REF_SEL_PORT       GPIO_PORT_XD            // CLK3_REF_SEL         A4
#define CLK2_REF_SEL_PORT       GPIO_PORT_XD            // CLK2_REF_SEL         A5
#define CLK1_REF_SEL_PORT       GPIO_PORT_XD            // CLK1_REF_SEL         A6
#define uRTM_PS_PORT            GPIO_PORT_XD            // uRTM_PS              A7
#define REFCLK1_PD_PORT         GPIO_PORT_XD            // REFCLK1_PD           B0
#define REFCLK2_PD_PORT         GPIO_PORT_XD            // REFCLK2_PD           B1
#define REFCLK3_PD_PORT         GPIO_PORT_XD            // REFCLK3_PD           B2
#define PLL_LOCK1_PORT          GPIO_PORT_XD            // PLL_LOCK1            B3
#define PLL_LOCK2_PORT          GPIO_PORT_XD            // PLL_LOCK2            B4
#define PLL_LOCK3_PORT          GPIO_PORT_XD            // PLL_LOCK3            B5
#define CLK_RSTz_PORT           GPIO_PORT_XD            // CLK_RSTz             B6
//#define NC                    GPIO_PORT_XD            // No Connect           B7

/* Common Ports */
#define SOC_GPIO_PORT           (GPIO_PORT_XA | GPIO_PORT_XB)
#define SOC_LOW_GPIO_PORT       GPIO_PORT_XA
#define SOC_HIGH_GPIO_PORT      GPIO_PORT_XB
#define DIP_SW_PORT             GPIO_PORT_H
#define SOC_CORESEL_PORT        GPIO_PORT_XC

/* GPIO Pin Abstractions */
/* PORT A */
#define U0_RX                   PIN_0                   // UART0 Receive        PA0
#define U0_TX                   PIN_1                   // UART0 Transmit       PA1
#define SPI0_CLK                PIN_2                   // SPI0 CLK             PA2
#define SPI0_CS0                PIN_3                   // SPI0 CS0             PA3
#define SPI0_MISO               PIN_4                   // SPI0 MISO            PA4
#define SPI0_MOSI               PIN_5                   // SPI0 MOSI            PA5
#define I2C1_SCL                PIN_6                   // I2C1 CLOCK           PA6
#define I2C1_SDA                PIN_7                   // I2C1 DATA            PA7
/* PORT B */
#define U1_RX                   PIN_0                   // UART1 Receive        PB0
#define U1_TX                   PIN_1                   // UART1 Transmit       PB1
#define I2C0_SCL                PIN_2                   // I2C0 CLOCK           PB2
#define I2C0_SDA                PIN_3                   // I2C0 DATA            PB3
#define MMC_GA0                 PIN_4                   // MMC GA0              PB4
#define MMC_GA1                 PIN_5                   // MMC GA1              PB5
#define MMC_GA2                 PIN_6                   // MMC GA2              PB6
#define MMC_GAPU                PIN_7                   // MMC GAPU             PB7
/* PORT C */
#define JTAG_TCK                PIN_0                   // JTAG                 PC0
#define JTAG_TMS                PIN_1                   // JTAG                 PC1
#define JTAG_TDI                PIN_2                   // JTAG                 PC2
#define JTAG_TDO                PIN_3                   // JTAG                 PC3
#define SPI0_CS1                PIN_4                   // SPI0 CS1             PC4
#define SPI0_CS2                PIN_5                   // SPI0 CS2             PC5
#define SPI0_CS3                PIN_6                   // SPI0 CS3             PC6
#define SPI0_CS4                PIN_7                   // SPI0 CS4             PC7
/* PORT D */
#define MAIN_POWER_START        PIN_0                   //                      PD0
#define MAIN_POWER_GOOD         PIN_1                   //                      PD1
#define SOC_POWER_START         PIN_2                   //                      PD2
#define SOC_POWER_GOOD          PIN_3                   //                      PD3
//#define NC                    PIN_4                   // NC                   PD4
//#define NC                    PIN_5                   // NC                   PD5
#define VCC3V3_MP_ALT_DET       PIN_6                   //                      PD6
#define LCD_RST                 PIN_7                   // LCD Reset            PD7
/* PORT E */
#define SPI1_CLK                PIN_0                   // SPI1 CLK             PE0
#define SPI1_CS0                PIN_1                   // SPI1 CS0             PE1
#define SPI1_MISO               PIN_2                   // SPI1 MISO            PE2
#define SPI1_MOSI               PIN_3                   // SPI1 MOSI            PE3
#define SOC_I2C_EN              PIN_4                   // SOC I2C Enable       PE4
#define PW_SEQ_I2C_EN           PIN_5                   // PW Seq I2C Enable    PE5
#define PW_SEQ_RST              PIN_6                   // PW Seq Reset         PE6
#define PHY_RST                 PIN_7                   // PHY Reset            PE7
/* PORT F */
//#define NC                    PIN_0                   // NC                   PF0
#define LCD_A0                  PIN_1                   // LCD A0               PF1
#define SPI0_CS5                PIN_2                   // SPI0 CS5             PF2
#define SPI0_CS6                PIN_3                   // SPI0 CS6             PF3
#define SPI0_GPIO_INT0          PIN_4                   // SPI0 GPIO/INT0       PF4
#define SPI0_GPIO_INT1          PIN_5                   // SPI0 GPIO/INT1       PF5
#define SPI0_GPIO_INT2          PIN_6                   // SPI0 GPIO/INT2       PF6
#define SPI0_GPIO_INT3          PIN_7                   // SPI0 GPIO/INT3       PF7
/* PORT G */
#define MMC_ENABLE_N            PIN_0                   //                      PG0
#define NOR_WP                  PIN_1                   //                      PG1
#define PCIECLK_MCU_PD          PIN_2                   //                      PG2
#define PCIECLK_OE              PIN_3                   //                      PG3
#define PMBUS_CTL               PIN_4                   //                      PG4
#define PMBUS_ALERT             PIN_5                   //                      PG5
#define SPI_GPIO_RESET          PIN_6                   // SPI GPIO Reset       PG6
#define MCU_BOOTSELECT          PIN_7                   // MCU bootselect       PG7
/* PORT H */
#define DIP_SW_B0               PIN_0                   // DIP Switch 0         PH0
#define DIP_SW_B1               PIN_1                   // DIP Switch 1         PH1
#define DIP_SW_B2               PIN_2                   // DIP Switch 2         PH2
#define DIP_SW_B3               PIN_3                   // DIP Switch 3         PH3
#define PCIECLK_MUX_SEL         PIN_4                   //                      PH4
#define MMC_PS_N0               PIN_5                   //                      PH5
#define MMC_RED_LED             PIN_6                   //                      PH6
#define MMC_BLUE_LED            PIN_7                   //                      PH7
/* PORT J */
#define WARM_RESET              PIN_0                   // Warm Reset           PJ0
#define FULL_RESET              PIN_1                   // Full Reset           PJ1
#define TRGRSTZ                 PIN_2                   //                      PJ2
//#define NC                    PIN_3                   // SPI0 CS6             PJ3
//#define NC                    PIN_4                   // SPI0 GPIO/INT0       PJ4
//#define NC                    PIN_5                   // SPI0 GPIO/INT1       PJ5
//#define NC                    PIN_6                   // SPI0 GPIO/INT2       PJ6
//#define NC                    PIN_7                   // SPI0 GPIO/INT3       PJ7

/* Port XA / CS3 */
#define SOC_GPIO_00             PIN_0                   // SOC GPIO 0           A0
#define SOC_GPIO_01             PIN_1                   // SOC GPIO 1           A1
#define SOC_GPIO_02             PIN_2                   // SOC GPIO 2           A2
#define SOC_GPIO_03             PIN_3                   // SOC GPIO 3           A3
#define SOC_GPIO_04             PIN_4                   // SOC GPIO 4           A4
#define SOC_GPIO_05             PIN_5                   // SOC GPIO 5           A5
#define SOC_GPIO_06             PIN_6                   // SOC GPIO 6           A6
#define SOC_GPIO_07             PIN_7                   // SOC GPIO 7           A7
#define SOC_GPIO_08             PIN_8                   // SOC GPIO 8           B0
#define SOC_GPIO_09             PIN_9                   // SOC GPIO 9           B1
#define SOC_GPIO_10             PIN_A                   // SOC GPIO 10          B2
#define SOC_GPIO_11             PIN_B                   // SOC GPIO 11          B3
#define SOC_GPIO_12             PIN_C                   // SOC GPIO 12          B4
#define SOC_GPIO_13             PIN_D                   // SOC GPIO 13          B5
#define SOC_GPIO_14             PIN_E                   // SOC GPIO 14          B6
#define SOC_GPIO_15             PIN_F                   // SOC GPIO 15          B7
/* Port XB / CS4 */
#define EMIF_OEZ                PIN_0                   // No Connect           A0
#define EMIF_DIR                PIN_1                   // No Connect           A1
#define MCU_GPS_ENABLE          PIN_2                   // MCU_GPS_ENABLE       A2
#define SOC_VPPB_EN             PIN_3                   // SOC_VPPB_EN          A3
#define SOC_GPIO_16             PIN_4                   // SOC GPIO 16          A4
#define PLLLOCK_LED             PIN_5                   // PLLLOCK LED          A5
#define SOC_PLLLOCK             PIN_6                   // SOC PLLLOCK          A6
//#define NC                    PIN_7                   // No Connect           A7
#define NAND_WPz                PIN_8                   // NAND WP              B0
#define EEPROM_WP               PIN_9                   // EEPROM WP            B1
#define BD_PRESENT              PIN_A                   // BD_PRESENT           B2
#define BD_ID0                  PIN_B                   // BD_ID0               B3
#define BD_ID1                  PIN_C                   // BD_ID1               B4
#define BD_ID2                  PIN_D                   // BD_ID2               B5
#define SOC_SCL1                PIN_E                   // SOC SCL              B6
#define SOC_SDA1                PIN_F                   // SOC SDA              B7
/* Port XC / CS5 */
#define SOC_CORECLKSEL          PIN_0                   // SOC_CORECLKSEL       A0
#define SOC_PACLKSEL            PIN_1                   // SOC_PACLKSEL         A1
#define SOC_HOUT                PIN_2                   // SOC_HOUT             A2
#define SOC_NMIZ                PIN_3                   // SOC_NMIZ             A3
#define SOC_LRESETZ             PIN_4                   // SOC_LRESETZ          A4
#define SOC_LRESETNMIENZ        PIN_5                   // SOC_LRESETNMIENZ     A5
#define SOC_BOOTCOMPLETE        PIN_6                   // SOC_BOOTCOMPLETE     A6
#define SOC_RESETSTATZ          PIN_7                   // SOC_RESETSTATZ       A7
#define SOC_CORESEL0            PIN_8                   // SOC_CORESEL0         B0
#define SOC_CORESEL1            PIN_9                   // SOC_CORESEL1         B1
#define SOC_CORESEL2            PIN_A                   // SOC_CORESEL2         B2
#define SOC_CORESEL3            PIN_B                   // SOC_CORESEL3         B3
#define TIMI_MUX_OEz            PIN_C                   // TIMI_MUX_OEz         B4
#define SOC_RESETFULLZ          PIN_D                   // SOC_RESETFULLZ       B5
#define SOC_RESETZ              PIN_E                   // SOC_RESETZ           B6
#define SOC_PORZ                PIN_F                   // SOC_PORZ             B7
/* Port XD / CS6 */
#define PHY_INT                 PIN_0                   // PHY_INT              A0
#define MCU_RESETSTATz          PIN_1                   // MCU_RESETSTATz       A1
#define EXT_PS                  PIN_2                   // EXT_PS               A2
#define MCU_EMU_DET             PIN_3                   // MCU_EMU_DET          A3
#define CLK3_REF_SEL            PIN_4                   // CLK3_REF_SEL         A4
#define CLK2_REF_SEL            PIN_5                   // CLK2_REF_SEL         A5
#define CLK1_REF_SEL            PIN_6                   // CLK1_REF_SEL         A6
#define uRTM_PS                 PIN_7                   // uRTM_PS              A7
#define REFCLK1_PD              PIN_8                   // REFCLK1_PD           B0
#define REFCLK2_PD              PIN_9                   // REFCLK2_PD           B1
#define REFCLK3_PD              PIN_A                   // REFCLK3_PD           B2
#define PLL_LOCK1               PIN_B                   // PLL_LOCK1            B3
#define PLL_LOCK2               PIN_C                   // PLL_LOCK2            B4
#define PLL_LOCK3               PIN_D                   // PLL_LOCK3            B5
#define CLK_RSTz                PIN_E                   // CLK_RSTz             B6
//#define NC                    PIN_F                   // No Connect           B7

/* Common Pins */
#define SOC_LOW_GPIO_PINS       (SOC_GPIO_00 |\
                                SOC_GPIO_01 |\
                                SOC_GPIO_02 |\
                                SOC_GPIO_03 |\
                                SOC_GPIO_04 |\
                                SOC_GPIO_05 |\
                                SOC_GPIO_06 |\
                                SOC_GPIO_07 |\
                                SOC_GPIO_08 |\
                                SOC_GPIO_09 |\
                                SOC_GPIO_10 |\
                                SOC_GPIO_11 |\
                                SOC_GPIO_12 |\
                                SOC_GPIO_13 |\
                                SOC_GPIO_14 |\
                                SOC_GPIO_15)
#define SOC_HIGH_GPIO_PINS      SOC_GPIO_16
#define DIP_SW_PINS             (DIP_SW_B0 | DIP_SW_B1 | DIP_SW_B2 | DIP_SW_B3)
#define SOC_CORESEL_PINS        (SOC_CORESEL0 | SOC_CORESEL1 | SOC_CORESEL2 | SOC_CORESEL3)

/* Abstracted Peripheral Bases */
// UART 0
#define CONSOLE_UART_BASE       UART0_BASE              // Console UART Base
// UART 1
#define SOC_UART_BASE           UART1_BASE              // SOC UART Base

/* Abstracted Pins */
// UART 0
#define CONSOLE_UART_RX     U0_RX                       // Console UART RX
#define CONSOLE_UART_TX     U0_TX                       // Console UART TX
// UART 1
#define SOC_UART_RX         U1_RX                       // SOC UART RX
#define SOC_UART_TX         U1_TX                       // SOC UART TX

/* Configuration Pin Defines */
#define GPIO_CONFIG_U0RX        GPIO_PA0_U0RX
#define GPIO_CONFIG_U0TX        GPIO_PA1_U0TX
#define GPIO_CONFIG_U1RX        GPIO_PB0_U1RX
#define GPIO_CONFIG_U1TX        GPIO_PB1_U1TX
#define GPIO_CONFIG_SSI0CLK     GPIO_PA2_SSI0CLK
#define GPIO_CONFIG_SSI0RX      GPIO_PA4_SSI0RX
#define GPIO_CONFIG_SSI0TX      GPIO_PA5_SSI0TX
#define GPIO_CONFIG_SSI1CLK     GPIO_PE0_SSI1CLK
#define GPIO_CONFIG_SSI1RX      GPIO_PE2_SSI1RX
#define GPIO_CONFIG_SSI1TX      GPIO_PE3_SSI1TX
#define GPIO_CONFIG_I2C0SCL     GPIO_PB2_I2C0SCL
#define GPIO_CONFIG_I2C0SDA     GPIO_PB3_I2C0SDA
#define GPIO_CONFIG_I2C1SCL     GPIO_PA6_I2C1SCL
#define GPIO_CONFIG_I2C1SDA     GPIO_PA7_I2C1SDA

/* ==================== Function Prototypes ================== */


#endif /* BMC_MAP_H_ */
