/*
******************************************************************************
* @file    ilps22qs_reg.c
* @author  Sensors Software Solution Team
* @brief   ILPS22QS driver file
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under BSD 3-Clause license,
* the "License"; You may not use this file except in compliance with the
* License. You may obtain a copy of the License at:
*                        opensource.org/licenses/BSD-3-Clause
*
******************************************************************************
*/

#include "ilps22qs_reg.h"

#define SPIDELAY 50 //ms
#define    BOOT_TIME         10 //ms

/**
 * @defgroup    ILPS22QS
 * @brief       This file provides a set of functions needed to drive the
 *              ilps22qs nano pressure sensor.
 * @{
 *
 */

/**
 * @defgroup    Interfaces_Functions
 * @brief       This section provide a set of functions used to read and
 *              write a generic register of the device.
 *              MANDATORY: return 0 -> no Error.
 * @{
 *
 */

/**
 * @brief  Read generic device register
 *
 * @param  ctx   read / write interface definitions(ptr)
 * @param  reg   register to read
 * @param  data  pointer to buffer that store the data read(ptr)
 * @param  len   number of consecutive register to read
 * @retval       interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t ilps22qs_read_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t *bufp,
                          uint16_t len, uint8_t w)
{

  // Drop CS, ALL chips
  gpio_put(ctx->sclkPin, 1);
  gpio_put(ctx->csPin, 0);
  //  sleep_us(1);
  
  //this is a read so set the R bit
  reg |= 0x80;
  //  printf("Bit banging %x",reg);
  //bitbang register to SDIO
  gpio_set_dir_out_masked        (       ctx->dataPinMask);  
  //   gpio_set_dir(sdio0Pin, GPIO_OUT); 
  /* gpio_set_dir(sdio1Pin, GPIO_OUT);     */
  for (uint8_t m = (uint8_t)0x80 ; m != 0; m >>= 1){
    gpio_put(ctx->sclkPin,0); 
    if (reg & m )
      gpio_set_mask(ctx->dataPinMask);
    else
      gpio_clr_mask(ctx->dataPinMask);

    //    printf("%x\n",reg & m);
    //     gpio_put(sdio0Pin, reg & m);
    //    gpio_put(sdio0Pin, 0); 
    /* gpio_put(sdio1Pin, reg & m);     */
    sleep_us(SPIDELAY);    
    gpio_put(ctx->sclkPin,1);
    sleep_us(SPIDELAY);    
  }

  //bitbang rest to SDIO
  //   gpio_set_dir(sdio0Pin, GPIO_IN); 
  //  gpio_set_dir(sdio1Pin, GPIO_IN);

  gpio_set_dir_in_masked        (       ctx->dataPinMask);  

  //  sleep_us(10);
  for (uint8_t i = 0; i < len; i++){
    bufp[i] = 0 ;
    for (uint8_t j = 8; j--;){
      gpio_put(ctx->sclkPin,0);
      sleep_us(SPIDELAY);            
      //            bufp[i] |= (gpio_get(sdio0Pin)<<j);
      bufp[i] |= (gpio_get(w)<<j);      
      gpio_put(ctx->sclkPin,1);
      sleep_us(SPIDELAY);      
    }
  }

  gpio_put(ctx->csPin, 1);


  
  return 0;


}

/**
 * @brief  Write generic device register
 *
 * @param  ctx   read / write interface definitions(ptr)
 * @param  reg   register to write
 * @param  data  pointer to data to write in register reg(ptr)
 * @param  len   number of consecutive register to write
 * @retval       interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t ilps22qs_write_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t *bufp,
                           uint16_t len)
{

  gpio_set_dir_out_masked        (       ctx->dataPinMask);

  //  gpio_set_dir(sdio0Pin,GPIO_OUT);

  
  // Drop CS, ALL chips
  gpio_put(ctx->sclkPin, 1);
  gpio_put(ctx->csPin, 0);

  //bitbang register to SDIO 
  for (uint8_t m = (uint8_t)0x80 ; m != 0; m >>= 1){
    gpio_put(ctx->sclkPin,0);
    if (reg & m )
      gpio_set_mask(ctx->dataPinMask);
    else
      gpio_clr_mask(ctx->dataPinMask);
    //    gpio_put(sdio0Pin, reg & m); 
    /* gpio_put(sdio1Pin, reg & m);     */
    sleep_us(SPIDELAY);
    gpio_put(ctx->sclkPin,1);
    sleep_us(SPIDELAY);    
  }

  //bitbang rest to SDIO
  for (uint8_t i = 0; i < len; i++){
    for (uint8_t m = (uint8_t)0x80 ; m != 0; m >>= 1){
      gpio_put(ctx->sclkPin,0);
      if (bufp[i] & m )
	gpio_set_mask(ctx->dataPinMask);
      else
	gpio_clr_mask(ctx->dataPinMask);
      
      //       gpio_put(sdio0Pin, bufp[i] & m); 
      /* gpio_put(sdio1Pin, bufp[i] & m); */
      sleep_us(SPIDELAY);
      gpio_put(ctx->sclkPin,1);
      sleep_us(SPIDELAY);
    }
  }

  gpio_put(ctx->csPin, 1);

  return 0;
  
}

/**
 * @}
 *
 */



/*
 * @brief  init function for ILP device
 *
 * @param  stmdev_ctx_t device pointer
 *
 */
int32_t ilp22qs_init(stmdev_ctx_t* dev_ctx, uint8_t sclkPin, uint8_t csPin,uint32_t dataPinMask ){


  ilps22qs_bus_mode_t bus_mode;
  ilps22qs_stat_t status;
  ilps22qs_id_t id;


  dev_ctx->sclkPin = sclkPin;
  dev_ctx->csPin = csPin;
  dev_ctx->dataPinMask = dataPinMask;


  
  /* Initialize platform specific hardware */

  // CS line
  gpio_init(dev_ctx->csPin);
  gpio_set_slew_rate(dev_ctx->csPin, GPIO_SLEW_RATE_SLOW);
  gpio_set_drive_strength(dev_ctx->csPin, GPIO_DRIVE_STRENGTH_2MA);
  
  gpio_set_dir(dev_ctx->csPin, GPIO_OUT);
  gpio_put(dev_ctx->csPin, 1);

  // Clock line direct port access
  gpio_init(dev_ctx->sclkPin);
  gpio_set_slew_rate(dev_ctx->sclkPin, GPIO_SLEW_RATE_SLOW);
  gpio_set_drive_strength(dev_ctx->sclkPin, GPIO_DRIVE_STRENGTH_12MA);

  gpio_set_dir(dev_ctx->sclkPin, GPIO_OUT);
  gpio_put(dev_ctx->sclkPin, 0);


  gpio_init_mask(dev_ctx->dataPinMask);
  gpio_set_dir_out_masked        (       dev_ctx->dataPinMask);


    
  gpio_set_drive_strength(4, GPIO_DRIVE_STRENGTH_12MA) ;
  gpio_set_drive_strength(6, GPIO_DRIVE_STRENGTH_12MA);
  gpio_set_drive_strength(7, GPIO_DRIVE_STRENGTH_12MA);
  gpio_set_drive_strength(8, GPIO_DRIVE_STRENGTH_12MA);
  
  // SDIO line direct port access
  //   gpio_init(sdio0Pin); 
  //   gpio_set_dir(sdio0Pin, GPIO_OUT); 
  /* gpio_put(sdio0Pin, 0); */
  /* gpio_init(sdio1Pin); */
  /* gpio_set_dir(sdio1Pin, GPIO_OUT); */
  /* gpio_put(sdio1Pin, 0); */


  
  /* Wait sensor boot time */
  sleep_ms(BOOT_TIME);

  //  ilps22qs_reset(dev_ctx);
  //  sleep_us(10);


  //  while(1){
  bus_mode.interface = ILPS22QS_SPI_3W ;
  ilps22qs_bus_mode_set(dev_ctx, &bus_mode);
  //        sleep_ms(1000);
  //  }
  //  printf("Interface = %x\n",bus_mode.interface);

  
  /* ilps22qs_id_get(dev_ctx, &id, dataPin[1]);    */
  /* if (id.whoami != ILPS22QS_ID) */
  /*   while(1); */


  
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

  return 0;

}




/**
 * @defgroup  Private_functions
 * @brief     Section collect all the utility functions needed by APIs.
 * @{
 *
 */

static void bytecpy(uint8_t *target, uint8_t *source)
{
  if ((target != NULL) && (source != NULL))
    {
      *target = *source;
    }
}

/**
 * @}
 *
 */

/**
 * @defgroup    Sensitivity
 * @brief       These functions convert raw-data into engineering units.
 * @{
 *
 */

float_t ilps22qs_from_fs1260_to_hPa(int32_t lsb)
{
  return ((float_t)lsb / 1048576.0f);   /* 4096.0f * 256 */
}

float_t ilps22qs_from_fs4000_to_hPa(int32_t lsb)
{
  return ((float_t)lsb /  524288.0f);   /* 2048.0f * 256 */
}

float_t ilps22qs_from_lsb_to_celsius(int16_t lsb)
{
  return ((float_t)lsb / 100.0f);
}

/**
 * @}
 *
 */

/**
 * @defgroup    Basic functions
 * @brief       This section groups all the functions concerning device basic
 *              configuration.
 * @{
 *
 */

/**
 * @brief  Device "Who am I".[get]
 *
 * @param  ctx   communication interface handler.(ptr)
 * @param  val   ID values.(ptr)
 * @retval       interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t ilps22qs_id_get(stmdev_ctx_t *ctx, ilps22qs_id_t *val, uint8_t w)
{
  uint8_t reg;
  int32_t ret;

  ret = ilps22qs_read_reg(ctx, ILPS22QS_WHO_AM_I, &reg, 1, w);
  val->whoami = reg;

  return ret;
}

/**
 * @brief  Configures the bus operating mode.[set]
 *
 * @param  ctx   communication interface handler.(ptr)
 * @param  val   configures the bus operating mode.(ptr)
 * @retval       interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t ilps22qs_bus_mode_set(stmdev_ctx_t *ctx, ilps22qs_bus_mode_t *val)
{
  //  ilps22qs_i3c_if_ctrl_add_t i3c_if_ctrl_add;
  //  ilps22qs_if_ctrl_t if_ctrl;
  int32_t ret;

  //  memset(&if_ctrl, 0, sizeof(ilps22qs_if_ctrl_t));


  uint8_t if_ctrl = 0x60;
  //  if_ctrl.i2c_i3c_dis = ((uint8_t)val->interface & 0x02U) >> 1;
  //  if_ctrl.en_spi_read = ((uint8_t)val->interface & 0x01U);
  //  printf("TEST=%x\n",if_ctrl);
  //  ret = ilps22qs_write_reg(ctx, ILPS22QS_IF_CTRL, (uint8_t *)&if_ctrl, 1);
  ret = ilps22qs_write_reg(ctx, ILPS22QS_IF_CTRL, &if_ctrl, 1);

  //  memset(&i3c_if_ctrl_add, 0, sizeof(ilps22qs_i3c_if_ctrl_add_t));
  
  //  i3c_if_ctrl_add.asf_on = (uint8_t)val->filter & 0x01U;
  //  ret = ilps22qs_write_reg(ctx, ILPS22QS_I3C_IF_CTRL_ADD,
  //                             (uint8_t *)&i3c_if_ctrl_add, 1);
  sleep_us(20);
  uint8_t if_ctrl_add = 0xA0;
  ret = ilps22qs_write_reg(ctx, ILPS22QS_I3C_IF_CTRL_ADD, &if_ctrl_add, 1);
  return ret;
}



int32_t ilps22qs_reset(stmdev_ctx_t *ctx){

  ilps22qs_ctrl_reg2_t ctrl_reg2;
  ilps22qs_ctrl_reg3_t ctrl_reg3;
  uint8_t reg[2];
  int32_t ret;

  memset(&ctrl_reg2, 0, sizeof(ilps22qs_ctrl_reg2_t));
  ctrl_reg2.swreset = PROPERTY_ENABLE;
  
  ret = ilps22qs_write_reg(ctx, ILPS22QS_CTRL_REG2,
			   (uint8_t *)&ctrl_reg2, 1);
}



/**
 * @brief  Get the status of the device.[get]
 *
 * @param  ctx   communication interface handler.(ptr)
 * @param  val   the status of the device.(ptr)
 * @retval       interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t ilps22qs_status_get(stmdev_ctx_t *ctx, ilps22qs_stat_t *val, uint8_t w)
{
  ilps22qs_interrupt_cfg_t interrupt_cfg;
  ilps22qs_int_source_t int_source;
  ilps22qs_ctrl_reg2_t ctrl_reg2;
  ilps22qs_status_t status;
  int32_t ret;

  ret = ilps22qs_read_reg(ctx, ILPS22QS_CTRL_REG2,
                          (uint8_t *)&ctrl_reg2, 1, w);
  if (ret == 0)
    {
      ret = ilps22qs_read_reg(ctx, ILPS22QS_INT_SOURCE, (uint8_t *)&int_source, 1, w);
    }
  if (ret == 0)
    {
      ret = ilps22qs_read_reg(ctx, ILPS22QS_STATUS, (uint8_t *)&status, 1, w);
    }
  if (ret == 0)
    {
      ret = ilps22qs_read_reg(ctx, ILPS22QS_INTERRUPT_CFG,
			      (uint8_t *)&interrupt_cfg, 1, w);
    }
  val->sw_reset  = ctrl_reg2.swreset;
  val->boot      = int_source.boot_on;
  val->drdy_pres = status.p_da;
  val->drdy_temp = status.t_da;
  val->ovr_pres  = status.p_or;
  val->ovr_temp  = status.t_or;
  val->end_meas  = ~ctrl_reg2.oneshot;
  val->ref_done = ~interrupt_cfg.autozero;

  return ret;
}


/**
 * @brief  Get the status of all the interrupt sources.[get]
 *
 * @param  ctx   communication interface handler.(ptr)
 * @param  val   the status of all the interrupt sources.(ptr)
 * @retval       interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t ilps22qs_all_sources_get(stmdev_ctx_t *ctx,
                                 ilps22qs_all_sources_t *val, uint8_t w)
{
  ilps22qs_fifo_status2_t fifo_status2;
  ilps22qs_int_source_t int_source;
  ilps22qs_status_t status;
  int32_t ret;

  ret = ilps22qs_read_reg(ctx, ILPS22QS_STATUS, (uint8_t *)&status, 1, w);
  if (ret == 0)
    {
      ret = ilps22qs_read_reg(ctx, ILPS22QS_INT_SOURCE,
			      (uint8_t *)&int_source, 1, w);
    }
  if (ret == 0)
    {
      ret = ilps22qs_read_reg(ctx, ILPS22QS_FIFO_STATUS2,
			      (uint8_t *)&fifo_status2, 1, w);
    }

  val->drdy_pres        = status.p_da;
  val->drdy_temp        = status.t_da;
  val->over_pres        = int_source.ph;
  val->under_pres       = int_source.pl;
  val->thrsld_pres      = int_source.ia;
  val->fifo_full        = fifo_status2.fifo_full_ia;
  val->fifo_ovr         = fifo_status2.fifo_ovr_ia;
  val->fifo_th          = fifo_status2.fifo_wtm_ia;

  return ret;
}


/**
 * @brief  Sensor conversion parameters selection.[set]
 *
 * @param  ctx   communication interface handler.(ptr)
 * @param  val   set the sensor conversion parameters.(ptr)
 * @retval       interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t ilps22qs_mode_set(stmdev_ctx_t *ctx, ilps22qs_md_t *val)
{
  ilps22qs_ctrl_reg1_t ctrl_reg1;
  ilps22qs_ctrl_reg2_t ctrl_reg2;
  uint8_t reg[2];
  int32_t ret;

  memset(&ctrl_reg1, 0, sizeof(ilps22qs_ctrl_reg1_t));
  memset(&ctrl_reg2, 0, sizeof(ilps22qs_ctrl_reg2_t));  

  

  ctrl_reg1.odr = (uint8_t)val->odr;
  ctrl_reg1.avg = (uint8_t)val->avg;
  ctrl_reg2.en_lpfp = (uint8_t)val->lpf & 0x01U;
  ctrl_reg2.lfpf_cfg = ((uint8_t)val->lpf & 0x02U) >> 2;
  ctrl_reg2.fs_mode = (uint8_t)val->fs;

  bytecpy(&reg[0], (uint8_t *)&ctrl_reg1);
  bytecpy(&reg[1], (uint8_t *)&ctrl_reg2);
  ret = ilps22qs_write_reg(ctx, ILPS22QS_CTRL_REG1, reg, 2);

  return ret;
}

/**
 * @brief  Sensor conversion parameters selection.[get]
 *
 * @param  ctx   communication interface handler.(ptr)
 * @param  val   get the sensor conversion parameters.(ptr)
 * @retval       interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t ilps22qs_mode_get(stmdev_ctx_t *ctx, ilps22qs_md_t *val, uint8_t w)
{
  ilps22qs_ctrl_reg1_t ctrl_reg1;
  ilps22qs_ctrl_reg2_t ctrl_reg2;
  uint8_t reg[2];
  int32_t ret;

  ret = ilps22qs_read_reg(ctx, ILPS22QS_CTRL_REG1, reg, 2, w);

  if (ret == 0)
    {
      bytecpy((uint8_t *)&ctrl_reg1, &reg[0]);
      bytecpy((uint8_t *)&ctrl_reg2, &reg[1]);

      switch (ctrl_reg2.fs_mode)
	{
	case ILPS22QS_1260hPa:
	  val->fs = ILPS22QS_1260hPa;
	  break;
	case ILPS22QS_4060hPa:
	  val->fs = ILPS22QS_4060hPa;
	  break;
	default:
	  val->fs = ILPS22QS_1260hPa;
	  break;
	}

      switch (ctrl_reg1.odr)
	{
	case ILPS22QS_ONE_SHOT:
	  val->odr = ILPS22QS_ONE_SHOT;
	  break;
	case ILPS22QS_1Hz:
	  val->odr = ILPS22QS_1Hz;
	  break;
	case ILPS22QS_4Hz:
	  val->odr = ILPS22QS_4Hz;
	  break;
	case ILPS22QS_10Hz:
	  val->odr = ILPS22QS_10Hz;
	  break;
	case ILPS22QS_25Hz:
	  val->odr = ILPS22QS_25Hz;
	  break;
	case ILPS22QS_50Hz:
	  val->odr = ILPS22QS_50Hz;
	  break;
	case ILPS22QS_75Hz:
	  val->odr = ILPS22QS_75Hz;
	  break;
	case ILPS22QS_100Hz:
	  val->odr = ILPS22QS_100Hz;
	  break;
	case ILPS22QS_200Hz:
	  val->odr = ILPS22QS_200Hz;
	  break;
	default:
	  val->odr = ILPS22QS_ONE_SHOT;
	  break;
	}

      switch (ctrl_reg1.avg)
	{
	case ILPS22QS_4_AVG:
	  val->avg = ILPS22QS_4_AVG;
	  break;
	case ILPS22QS_8_AVG:
	  val->avg = ILPS22QS_8_AVG;
	  break;
	case ILPS22QS_16_AVG:
	  val->avg = ILPS22QS_16_AVG;
	  break;
	case ILPS22QS_32_AVG:
	  val->avg = ILPS22QS_32_AVG;
	  break;
	case ILPS22QS_64_AVG:
	  val->avg = ILPS22QS_64_AVG;
	  break;
	case ILPS22QS_128_AVG:
	  val->avg = ILPS22QS_128_AVG;
	  break;
	case ILPS22QS_256_AVG:
	  val->avg = ILPS22QS_256_AVG;
	  break;
	case ILPS22QS_512_AVG:
	  val->avg = ILPS22QS_512_AVG;
	  break;
	default:
	  val->avg = ILPS22QS_4_AVG;
	  break;
	}

      switch ((ctrl_reg2.lfpf_cfg << 2) | ctrl_reg2.en_lpfp)
	{
	case ILPS22QS_LPF_DISABLE:
	  val->lpf = ILPS22QS_LPF_DISABLE;
	  break;
	case ILPS22QS_LPF_ODR_DIV_4:
	  val->lpf = ILPS22QS_LPF_ODR_DIV_4;
	  break;
	case ILPS22QS_LPF_ODR_DIV_9:
	  val->lpf = ILPS22QS_LPF_ODR_DIV_9;
	  break;
	default:
	  val->lpf = ILPS22QS_LPF_DISABLE;
	  break;
	}
    }
  return ret;
}


int32_t ilps22qs_data_get(stmdev_ctx_t *ctx, ilps22qs_md_t *md,
                          ilps22qs_data_t *data, uint8_t w)
{
  uint8_t buff[5];
  int32_t ret;

  ret = ilps22qs_read_reg(ctx, ILPS22QS_PRESS_OUT_XL, buff, 5, w);

  /* pressure conversion */
  data->pressure.raw = (int32_t)buff[2];
  data->pressure.raw = (data->pressure.raw * 256) + (int32_t) buff[1];
  data->pressure.raw = (data->pressure.raw * 256) + (int32_t) buff[0];
  data->pressure.raw = data->pressure.raw * 256;

  switch (md->fs)
    {
    case ILPS22QS_1260hPa:
      data->pressure.hpa = ilps22qs_from_fs1260_to_hPa(data->pressure.raw);
      break;
    case ILPS22QS_4060hPa:
      data->pressure.hpa = ilps22qs_from_fs4000_to_hPa(data->pressure.raw);
      break;
    default:
      data->pressure.hpa = 0.0f;
      break;
    }

  /* temperature conversion */
  data->heat.raw = (int16_t)buff[4];
  data->heat.raw = (data->heat.raw * 256) + (int16_t) buff[3];
  data->heat.deg_c = ilps22qs_from_lsb_to_celsius(data->heat.raw);

  return ret;
}


int32_t ilps22qs_ah_qvar_disable(stmdev_ctx_t *ctx)
{
  uint32_t val = 0;
  int32_t ret;

  ret = ilps22qs_write_reg(ctx, ILPS22QS_ANALOGIC_HUB_DISABLE, (uint8_t *)&val, 1);

  return ret;
}



/**
 * @}
 *
 */




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
