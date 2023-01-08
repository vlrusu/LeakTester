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


#define    MAXDATALINES      6


/* Private variables ---------------------------------------------------------*/

static ilps22qs_data_t data;

/* Extern variables ----------------------------------------------------------*/



/* Main Example --------------------------------------------------------------*/





/*
 * @brief  main 
 */


int main(){



  stdio_init_all();
  sleep_ms(1000);
  


  ilps22qs_all_sources_t all_sources;

  uint8_t sclkPin = 2;                                                // SPI clock
  //const uint8_t sclkPin = 19;                                                // SPI clock
  uint8_t csPin = 5;                                       // SPI Chip select
  //const uint8_t csPin = 21;                                       // SPI Chip select
  //4,6,7,8,9,10


  const uint8_t dataPin0[MAXDATALINES] = {4,6,7,8,9,10};
  const uint8_t dataPin1[MAXDATALINES] = {16,17,18,20,21,26};

  const uint8_t pinMask0 = 0b110011;
  const uint8_t pinMask1 = 0b000101;  
  
  //determine the masks
  uint32_t dataPinMask0 = 0;
  uint32_t dataPinMask1 = 0;  
  for (int i = 0; i < MAXDATALINES; i++){
    if (pinMask0 >> i & 0x1)
      dataPinMask0 |=  (1<<dataPin0[i]);
    if (pinMask1 >> i & 0x1)
      dataPinMask1 |=  (1<<dataPin1[i]);

  }

  stmdev_ctx_t dev_ctx0;
  stmdev_ctx_t dev_ctx1;  



  ilp22qs_init(&dev_ctx0,2,5,dataPinMask0);
  ilp22qs_init(&dev_ctx1,19,21,dataPinMask1);    

  /* Check device ID */
  ilps22qs_id_t id;

  
  while(1){
    for (int i = 0; i < MAXDATALINES; i++){
      if ( (dev_ctx0.dataPinMask & (1<<dataPin0[i]))){
  	ilps22qs_id_get(&dev_ctx0, &id,dataPin0[i]);
  	printf("Device 0 %d ID=%x\n",i,id.whoami);
      }
    }
    for (int i = 0; i < MAXDATALINES; i++){
      if ( (dev_ctx1.dataPinMask & (1<<dataPin1[i]))){
  	ilps22qs_id_get(&dev_ctx1, &id,dataPin1[i]);
  	printf("Device 1 %d ID=%x\n",i,id.whoami);
      }
    }
    sleep_ms(1000);
  }
			     

  /* Set Output Data Rate */
  ilps22qs_md_t md;  
  md.odr = ILPS22QS_4Hz;
  md.avg = ILPS22QS_16_AVG;
  md.lpf = ILPS22QS_LPF_ODR_DIV_4;
  md.fs = ILPS22QS_1260hPa;



  ilps22qs_mode_set(&dev_ctx0, &md);
  ilps22qs_mode_set(&dev_ctx1, &md);  






  /* Read samples in polling mode (no int) */
  while(1)
    {

      for (int idev = 0; idev < MAXDATALINES; idev++){
	/* Read output only if new values are available */
	if ( (dev_ctx0.dataPinMask & (1<<dataPin0[idev]))){

	  ilps22qs_all_sources_get(&dev_ctx0, &all_sources,dataPin0[idev]);	  
	  /* while ( (all_sources.drdy_pres | all_sources.drdy_temp) == 0){ */
	  /*   ilps22qs_all_sources_get(&dev_ctx0, &all_sources,dataPin0[idev]); */
	  /*   sleep_ms(1000); */
	  /* } */

	//    if ( all_sources.drdy_pres | all_sources.drdy_temp ) {
	  ilps22qs_data_get(&dev_ctx0, &md, &data,dataPin0[idev]);

	  printf(
		 //              "%d. %d pressure [hPa]:%6.2f temperature [degC]:%6.2f\r\n",
		 "%d %d %d %6.2f %6.2f ",	     
		 idev,all_sources.drdy_pres,all_sources.drdy_temp,data.pressure.hpa, data.heat.deg_c);
	  
	  //    }
	  sleep_ms(1000);
	}
      }
      printf("\r\n");
    }


  
}
