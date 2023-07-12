/**
 * @file main.c
 * @author Vadim Rusu (vadim.l.rusu@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-01-14
 *
 * @copyright Copyright (c) 2023
 *
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

#define MAXDATALINES 6
//#define DEBUG 
/* Private variables ---------------------------------------------------------*/

#define P1NAME "PLANE7"
#define P2NAME "PLANE11"

static ilps22qs_data_t data;

/**
 * @brief main
 *
 * @return int
 */
int main()
{

  stdio_init_all();
  sleep_ms(1000);

  ilps22qs_all_sources_t all_sources;
  /**
   * @brief dataPins used for SPI comms. There are two ports on the
   * PicoBase board, for now I use them both.
   *
   */
  //  const uint8_t dataPin0[MAXDATALINES] = {4, 6, 7, 8, 9, 10};
  const uint8_t dataPin0[MAXDATALINES] = {7,6,4,2,3,9};  
  //  const uint8_t dataPin1[MAXDATALINES] = {26, 27, 18, 16, 17, 22};
  const uint8_t dataPin1[MAXDATALINES] = {21,19,18,16,17,22};  
  const uint8_t clk0 = 5;
  const uint8_t clk1 = 20;
  const uint8_t cs0 = 10;
  const uint8_t cs1 = 27;  

  /**
   * @brief what sensors I want to look at. Basically a mask on the pins
   * before
   *
   */
  //  const uint8_t pinMask0 = 0b111111;
  const uint8_t pinMask0 = 0b111111;
  const uint8_t pinMask1 = 0b111111;

  /**
   * @brief determine the masks
   *
   */
  uint32_t dataPinMask0 = 0;
  uint32_t dataPinMask1 = 0;
  for (int i = 0; i < MAXDATALINES; i++)
  {
    if (pinMask0 >> i & 0x1)
      dataPinMask0 |= (1 << dataPin0[i]);
    if (pinMask1 >> i & 0x1)
      dataPinMask1 |= (1 << dataPin1[i]);
  }

  stmdev_ctx_t dev_ctx0;
  stmdev_ctx_t dev_ctx1;

  ilp22qs_init(&dev_ctx0, clk0, cs0, dataPinMask0);
  ilp22qs_init(&dev_ctx1, clk1, cs1, dataPinMask1);

  /* Check device ID */
  ilps22qs_id_t id;

#ifdef DEBUG
  while (1)
  {
    for (int i = 0; i < MAXDATALINES; i++)
    {
      if ((dev_ctx0.dataPinMask & (1 << dataPin0[i])))
      {
        ilps22qs_id_get(&dev_ctx0, &id, dataPin0[i]);
        printf("Device 0 %d ID=%x\n", i, id.whoami);
      }
    }
    for (int i = 0; i < MAXDATALINES; i++)
    {
      if ((dev_ctx1.dataPinMask & (1 << dataPin1[i])))
      {
        ilps22qs_id_get(&dev_ctx1, &id, dataPin1[i]);
        printf("Device 1 %d ID=%x\n", i, id.whoami);
      }
    }
    sleep_ms(1000);
  }
#endif

  /**
   * @brief set data rate, range, others
   *
   */
  ilps22qs_md_t md;
  md.odr = ILPS22QS_4Hz;
  md.avg = ILPS22QS_16_AVG;
  md.lpf = ILPS22QS_LPF_ODR_DIV_4;
  md.fs = ILPS22QS_4060hPa;

  ilps22qs_mode_set(&dev_ctx0, &md);
  ilps22qs_mode_set(&dev_ctx1, &md);

  /**
   * @brief read samples in polling mode
   * no checks here, so data may not be ready. This has to be dealt
   * with in the driver python script
   *
   */
  while (1)
  {


    // Process keyboard entry, if any

    int input = getchar_timeout_us(10);
    if (input == 'R') {
      printf("Resetting \n");
      ilp22qs_init(&dev_ctx0, clk0, cs0, dataPinMask0);
      ilp22qs_init(&dev_ctx1, clk1, cs1, dataPinMask1);

      //      ilp22qs_init(&dev_ctx0, 2, 5, dataPinMask0);
      //      ilp22qs_init(&dev_ctx1, 19, 21, dataPinMask1);

      ilps22qs_mode_set(&dev_ctx0, &md);
      ilps22qs_mode_set(&dev_ctx1, &md);
    }


    printf("%s ",P1NAME);
    for (int idev = 0; idev < MAXDATALINES; idev++)
      {
	/* Read output only if new values are available */
	if ((dev_ctx0.dataPinMask & (1 << dataPin0[idev])))
	  {
	    ilps22qs_id_get(&dev_ctx0, &id, dataPin0[idev]);
	    
	    ilps22qs_all_sources_get(&dev_ctx0, &all_sources, dataPin0[idev]);

	    ilps22qs_data_get(&dev_ctx0, &md, &data, dataPin0[idev]);
	    
	    printf(
		   "%d %6.2f %6.2f ",
		   idev, data.pressure.hpa, data.heat.deg_c);

	    sleep_ms(1000);
	  }
      }

    printf("%s ",P2NAME);
    
    for (int idev = 0; idev < MAXDATALINES; idev++)
      {
	if ((dev_ctx1.dataPinMask & (1 << dataPin1[idev])))
	  {

	    ilps22qs_all_sources_get(&dev_ctx1, &all_sources, dataPin1[idev]);
	    
	    ilps22qs_data_get(&dev_ctx1, &md, &data, dataPin1[idev]);
	    
	    printf(
		   "%d %6.2f %6.2f ",
		   idev, data.pressure.hpa, data.heat.deg_c);
	    
	    sleep_ms(1000);
	  }
      }
    printf("\r\n");
  }
}

