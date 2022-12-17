/*
 ******************************************************************************
 * @file    read_data_polling.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/*
/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */



/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ilps22qs_reg.h"
#include "pico/stdlib.h"
#include "pico/types.h"
#include "pico/platform.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"



/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME         10 //ms

/* Private variables ---------------------------------------------------------*/

static ilps22qs_data_t data;

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len, uint8_t w);
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/


/**
  * @defgroup    Pins for PICO
  * @brief      Define pins for PICO
  * @{
  *
  */
const uint8_t nSensors = 2;
const uint8_t sclkPin = 2;                                                // SPI clock
const uint8_t csPin = 5;                                       // SPI Chip select
const uint32_t dataPinMask = 0b1010000;
const uint8_t dataPin[2] = {4,6} ;
const uint8_t sdio0Pin = 4;                                       // SPI SDIO pin
const uint8_t sdio1Pin = 6;                                       // SPI SDIO pin
const uint8_t spidelay = 1;
/**
  * @}
  *
  */



/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{

  gpio_set_dir_out_masked        (       dataPinMask);

 // Drop CS, ALL chips
  gpio_put(sclkPin, 1);
  gpio_put(csPin, 0);

  //bitbang register to SDIO 
  for (uint8_t m = (uint8_t)0x80 ; m != 0; m >>= 1){
    gpio_put(sclkPin,0);
    if (reg & m )
      gpio_set_mask(dataPinMask);
    else
      gpio_clr_mask(dataPinMask);
    /* gpio_put(sdio0Pin, reg & m); */
    /* gpio_put(sdio1Pin, reg & m);     */
    sleep_us(spidelay);
    gpio_put(sclkPin,1);
    sleep_us(spidelay);    
  }

  //bitbang rest to SDIO
  for (uint8_t i = 0; i < len; i++){
    for (uint8_t m = (uint8_t)0x80 ; m != 0; m >>= 1){
      gpio_put(sclkPin,0);
      if (bufp[i] & m )
	gpio_set_mask(dataPinMask);
      else
	gpio_clr_mask(dataPinMask);
      
      /* gpio_put(sdio0Pin, bufp[i] & m); */
      /* gpio_put(sdio1Pin, bufp[i] & m); */
      sleep_us(spidelay);
      gpio_put(sclkPin,1);
      sleep_us(spidelay);
    }
  }

  gpio_put(csPin, 1);

  return 0;
}
  
 
/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len, uint8_t w)
{

 // Drop CS, ALL chips
  gpio_put(sclkPin, 1);
  gpio_put(csPin, 0);
  sleep_us(1);
  
  //this is a read so set the R bit
  reg |= 0x80;
  //  printf("Bit banging %x",reg);
  //bitbang register to SDIO
  gpio_set_dir_out_masked        (       dataPinMask);  
  /* gpio_set_dir(sdio0Pin, GPIO_OUT); */
  /* gpio_set_dir(sdio1Pin, GPIO_OUT);     */
  for (uint8_t m = (uint8_t)0x80 ; m != 0; m >>= 1){
    gpio_put(sclkPin,0);
    if (reg & m )
      gpio_set_mask(dataPinMask);
    else
      gpio_clr_mask(dataPinMask);
    
    /* gpio_put(sdio0Pin, reg & m); */
    /* gpio_put(sdio1Pin, reg & m);     */
    sleep_us(spidelay);    
    gpio_put(sclkPin,1);
    sleep_us(spidelay);    
  }

  //bitbang rest to SDIO
  /* gpio_set_dir(sdio0Pin, GPIO_IN); */
  /* gpio_set_dir(sdio1Pin, GPIO_IN);   */

  gpio_set_dir_in_masked        (       dataPinMask);  

  //  sleep_us(10);
  for (uint8_t i = 0; i < len; i++){
    bufp[i] = 0 ;
    for (uint8_t j = 8; j--;){
      gpio_put(sclkPin,0);
      sleep_us(spidelay);            
      //      bufp[i] |= (gpio_get(sdio1Pin)<<j);
      bufp[i] |= (gpio_get(w)<<j);      
      gpio_put(sclkPin,1);
      sleep_us(spidelay);      
    }
  }

  gpio_put(csPin, 1);


  
  return 0;
}


/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{

  sleep_ms(ms);
}









/*
 * @brief  platform specific initialization (platform dependent)
 * initialize pins, etc.
 */
static void platform_init(void)
{

 // CS line
  gpio_init(csPin);
  gpio_set_slew_rate(csPin, GPIO_SLEW_RATE_SLOW);
  gpio_set_drive_strength(csPin, GPIO_DRIVE_STRENGTH_8MA);
  
  gpio_set_dir(csPin, GPIO_OUT);
  gpio_put(csPin, 1);

  // Clock line direct port access
  gpio_init(sclkPin);
  gpio_set_slew_rate(sclkPin, GPIO_SLEW_RATE_SLOW);
  gpio_set_drive_strength(sclkPin, GPIO_DRIVE_STRENGTH_8MA);
  gpio_set_dir(sclkPin, GPIO_OUT);
  gpio_put(sclkPin, 0);


  gpio_init_mask(dataPinMask);
  gpio_set_dir_out_masked        (       dataPinMask);
  
  // SDIO line direct port access
  /* gpio_init(sdio0Pin); */
  /* gpio_set_dir(sdio0Pin, GPIO_OUT); */
  /* gpio_put(sdio0Pin, 0); */
  /* gpio_init(sdio1Pin); */
  /* gpio_set_dir(sdio1Pin, GPIO_OUT); */
  /* gpio_put(sdio1Pin, 0); */

  
}


/*
 * @brief  init function for ILP device
 *
 * @param  stmdev_ctx_t device pointer
 *
 */
void ilp22qs_init(stmdev_ctx_t* dev_ctx){


  ilps22qs_bus_mode_t bus_mode;
  ilps22qs_stat_t status;
  ilps22qs_id_t id;

    /* Initialize mems driver interface */
  dev_ctx->write_reg = platform_write;
  dev_ctx->read_reg = platform_read;
  dev_ctx->handle = 0;

  /* Initialize platform specific hardware */
  //  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  //  ilps22qs_reset(dev_ctx);
  //  sleep_us(10);


  //  while(1){
    bus_mode.interface = ILPS22QS_SPI_3W ;
    ilps22qs_bus_mode_set(dev_ctx, &bus_mode);
    //        sleep_ms(1000);
    //  }
    //  printf("Interface = %x\n",bus_mode.interface);

  
  /* Check device ID */

    //  while(1){
    ilps22qs_id_get(dev_ctx, &id,dataPin[1]);
    printf("Device ID=%x\n",id.whoami);
    sleep_ms(1000);
    //  }
  ilps22qs_id_get(dev_ctx, &id, dataPin[0]);   
  if (id.whoami != ILPS22QS_ID)
    while(1);


  
  /* Restore default configuration */
  /* ilps22qs_init_set(dev_ctx, ILPS22QS_RESET); */
  /* do { */
  /*   ilps22qs_status_get(dev_ctx, &status); */
  /* } while (status.sw_reset); */



  ilps22qs_reset(dev_ctx);

  
  ilps22qs_bus_mode_set(dev_ctx, &bus_mode);

  /* Disable AH/QVAR to save power consumption */
  ilps22qs_ah_qvar_disable(dev_ctx);

  /* Set bdu and if_inc recommended for driver usage */
  //  ilps22qs_init_set(dev_ctx, ILPS22QS_DRV_RDY);

  /* Select bus interface */
  bus_mode.filter = ILPS22QS_AUTO;
  bus_mode.interface = ILPS22QS_SPI_3W ;
  ilps22qs_bus_mode_set(dev_ctx, &bus_mode);


}

/*
 * @brief  main 
 */


int main(){



  platform_init();
  stdio_init_all();
  sleep_ms(1000);
  


  ilps22qs_all_sources_t all_sources;
  stmdev_ctx_t dev_ctx;

  ilps22qs_md_t md;

  ilp22qs_init(&dev_ctx);


    /* Set Output Data Rate */
  md.odr = ILPS22QS_4Hz;
  
  md.avg = ILPS22QS_16_AVG;
  md.lpf = ILPS22QS_LPF_ODR_DIV_4;
  md.fs = ILPS22QS_1260hPa;
  ilps22qs_mode_set(&dev_ctx, &md);

  /* Read samples in polling mode (no int) */
  while(1)
  {

    
    /* Read output only if new values are available */
    while ( (all_sources.drdy_pres | all_sources.drdy_temp) == 0){
      ilps22qs_all_sources_get(&dev_ctx, &all_sources,dataPin[0]);

      sleep_ms(1000);
    }

    //    if ( all_sources.drdy_pres | all_sources.drdy_temp ) {
      ilps22qs_data_get(&dev_ctx, &md, &data,dataPin[0]);

      printf(
              "1 . pressure [hPa]:%6.2f temperature [degC]:%6.2f\r\n",
              data.pressure.hpa, data.heat.deg_c);

      //    }
      sleep_ms(1000);

    /* Read output only if new values are available */
    while ( (all_sources.drdy_pres | all_sources.drdy_temp) == 0){
      ilps22qs_all_sources_get(&dev_ctx, &all_sources,dataPin[1]);

      sleep_ms(1000);
    }
      
    //    ilps22qs_all_sources_get(&dev_ctx, &all_sources,dataPin[1]);

    //    sleep_ms(1000);

    //    if ( all_sources.drdy_pres | all_sources.drdy_temp ) {
      ilps22qs_data_get(&dev_ctx, &md, &data,dataPin[1]);

      printf(
              "2 . pressure [hPa]:%6.2f temperature [degC]:%6.2f\r\n",
              data.pressure.hpa, data.heat.deg_c);

      //    }

      sleep_ms(1000);

  }



  
}
