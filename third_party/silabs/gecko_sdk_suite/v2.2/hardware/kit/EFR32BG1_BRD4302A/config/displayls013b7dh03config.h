/***************************************************************************//**
 * @file displayls013b7dh03config.h
 * @brief BRD4302A specific configuration for the display driver for
 *        the Sharp Memory LCD model LS013B7DH03.
 * @version 5.4.0
 *******************************************************************************
 * # License

 * <b>Copyright 2015 Silicon Labs, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#ifndef __SILICON_LABS_DISPLAY_LS013B7DH03_CONFIG_H_
#define __SILICON_LABS_DISPLAY_LS013B7DH03_CONFIG_H_

#include "displayconfigapp.h"

/* Display device name. */
#define SHARP_MEMLCD_DEVICE_NAME   "Sharp LS013B7DH03 #1"

/* LCD and SPI GPIO pin connections on the EFR32BG1_BRD4302A. */
#define LCD_PORT_SCLK           (gpioPortC)
#define LCD_PIN_SCLK            (6)
#define LCD_PORT_SI             (gpioPortC)
#define LCD_PIN_SI              (7)
#define LCD_PORT_SCS            (gpioPortD)
#define LCD_PIN_SCS             (14)
#define LCD_PORT_EXTCOMIN       (gpioPortD)
#define LCD_PIN_EXTCOMIN        (13)
#define LCD_PORT_DISP_SEL       (gpioPortD)
#define LCD_PIN_DISP_SEL        (15)
#define LCD_PORT_DISP_PWR       (gpioPortD)
#define LCD_PIN_DISP_PWR        (15)

/* PRS settings for polarity inversion extcomin auto toggle.  */
#define LCD_AUTO_TOGGLE_PRS_CH    (4)  /* PRS channel 4.      */
#define LCD_AUTO_TOGGLE_PRS_ROUTELOC()  PRS->ROUTELOC1 = \
  ((PRS->ROUTELOC1 & ~_PRS_ROUTELOC1_CH4LOC_MASK) | PRS_ROUTELOC1_CH4LOC_LOC4)
#define LCD_AUTO_TOGGLE_PRS_ROUTEPEN    PRS_ROUTEPEN_CH4PEN

/*
 * Select how LCD polarity inversion should be handled:
 *
 * If POLARITY_INVERSION_EXTCOMIN is defined,
 * the polarity inversion is armed for every rising edge of the EXTCOMIN
 * pin. The actual polarity inversion is triggered at the next transision of
 * SCS. This mode is recommended because it causes less CPU and SPI load than
 * the alternative mode, see below.
 * If POLARITY_INVERSION_EXTCOMIN is undefined,
 * the polarity inversion is toggled by sending an SPI command. This mode
 * causes more CPU and SPI load than using the EXTCOMIN pin mode.
 */
#define POLARITY_INVERSION_EXTCOMIN

/* Define POLARITY_INVERSION_EXTCOMIN_PAL_AUTO_TOGGLE if you want the PAL
 * (Platform Abstraction Layer interface) to automatically toggle the EXTCOMIN
 *  pin.
 * If the PAL_TIMER_REPEAT function is defined the EXTCOMIN toggling is handled
 * by a timer repeat system, therefore we must undefine
 * POLARITY_INVERSION_EXTCOMIN_PAL_AUTO_TOGGLE;
 */
#ifndef PAL_TIMER_REPEAT_FUNCTION
  #define POLARITY_INVERSION_EXTCOMIN_PAL_AUTO_TOGGLE
#endif
#endif /* __SILICON_LABS_DISPLAY_LS013B7DH03_CONFIG_H_ */
