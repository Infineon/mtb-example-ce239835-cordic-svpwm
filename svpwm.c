/******************************************************************************
* File Name:   svpwm.c
*
* Description: 7-segment standard space vector modulation implementation.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "svpwm.h"

#include "cy_device.h"
#include "cy_syslib.h"
#include "cycfg_peripherals.h"

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * LOCAL DATA
 **********************************************************************************************************************/


/***********************************************************************************************************************
 * LOCAL DATA
 **********************************************************************************************************************/


/**********************************************************************************************************************
* API IMPLEMENTATION
**********************************************************************************************************************/
/*
 * SVM Segment Time calculation using CORDIC
 */
static inline void SVPWM_TimecalcUsingCORDIC(SVPWM_t* const HandlePtr, uint16_t Amplitude, uint32_t Angle)
{
  uint32_t cosx_amp;
  uint32_t sinx_amp;
  uint32_t cosx;
  uint32_t sinx;
  uint32_t tacord_tmp;
  uint32_t tbcord_tmp;
  uint32_t Angle_tmp;
  uint16_t ta_tb;

  Angle_tmp =Angle & (uint32_t)0xFFFFFF;

  HandlePtr->sector = ((uint32_t)((Angle_tmp * 6U) >> 24U) & 7U);
  /*calculate sector angle*/
  HandlePtr->sector_angle = (uint32_t)((Angle_tmp) - (uint32_t)(HandlePtr->sector * SVPWM_SIXTYDEG_24BIT));

  if ((uint16_t)Amplitude > (HandlePtr->max_amplitude))
  {
    Amplitude = HandlePtr->max_amplitude;
  }

  /* Set CORDIC to Circular Operating Mode and Rotation Mode,
   * Auto start of calculation after write access to X parameter data register,
   * X result data format to Unsigned, when read,
   * After the last iteration of calculation the X & Y values are divided by 1
   */
  MXCORDIC->CON = (uint32_t)0x2A;
  
  /*Load the amplitude value */
  MXCORDIC->CORDX =  (uint32_t)(1024 << MXCORDIC_CORDX_DATA_Pos);
  
  /* Set the Y value to zero*/
  MXCORDIC->CORDY =  (uint32_t)(0U << MXCORDIC_CORDY_DATA_Pos);

  /*Input sector angle*/
  MXCORDIC->CORDZ = (uint32_t)(HandlePtr->sector_angle << ((uint32_t)MXCORDIC_CORDZ_DATA_Pos));


  /*Reading cordic X result which is costheta*/
  cosx_amp = (MXCORDIC->CORRX >> MXCORDIC_CORDX_DATA_Pos);
  cosx     = (cosx_amp * SVPWM_KCOSSCALE)>>10;
  
  /*Reading cordic Y result which is sintheta*/
  sinx_amp = (MXCORDIC->CORRY >> MXCORDIC_CORDY_DATA_Pos);
  sinx     = (sinx_amp * SVPWM_KSINSCALE)>>10;

  cosx_amp = cosx * Amplitude;
  sinx_amp = sinx * Amplitude;

   /* Multiply Result value with sinscale to eliminate gain factor K*/
   tacord_tmp = (uint32_t)(sinx_amp>>14U);
   HandlePtr->ta = (uint16_t)((tacord_tmp * (uint32_t)HandlePtr->amplitude_scale) >> 10U);

   /* Multiply Result value with cosscale to eliminate gain factor K*
    * sin(60-Angle) = (Sqrt(3)*cos(angle) - sin(angle))/2
    */
   tbcord_tmp = (uint32_t)((cosx_amp - sinx_amp))>>15U;
   HandlePtr->tb = (uint16_t)((tbcord_tmp * (uint32_t)HandlePtr->amplitude_scale) >> 10U);

  ta_tb = HandlePtr->ta + HandlePtr->tb;
  HandlePtr->t0 = HandlePtr->period - ta_tb;

}

/*******************************************************************************
 **                      Public Function Definitions                           **
 *******************************************************************************/
/**
 * This function initializes the peripherals and SVM structure 
 * required for the SVM algorithm.
 */
void SVPWM_Init(SVPWM_t* const HandlePtr)
{
    HandlePtr->ta                     = 0U;
    HandlePtr->tb                     = 0U;
    HandlePtr->v_ta                   = 0U;
    HandlePtr->v_tb                   = 0U;
    HandlePtr->sector                 = 0U;
    HandlePtr->period                 = PWM_U_config.period0;
    HandlePtr->tmin                   = 0U;
    HandlePtr->max_amplitude          = 16384U; //2^14 (Max dc link voltage)
    HandlePtr->t0                     = PWM_U_config.period0;
    HandlePtr->amplitude_scale        = (uint16_t)(((uint32_t)PWM_U_config.period0 * 1024U)/16384U);
}

/**
 * This is the SVM algorithm for sinusoidal commutation.
 * It updates the compare registers of the TCPWM as per calculated
 * duty cycle.
 */
void SVPWM_SVMUpdate(SVPWM_t* const HandlePtr, uint16_t Amplitude, uint32_t Angle)
{
  SVPWM_TimecalcUsingCORDIC(HandlePtr, (uint16_t) Amplitude, (uint32_t) Angle);

  /*If segment time ta less than tmin set to tmin*/
  if (HandlePtr->ta < (uint16_t) HandlePtr->tmin)
  {
    HandlePtr->ta = (uint16_t) HandlePtr->tmin;
  }
  /*If segment time tb less than tmin set to tmin*/
  if (HandlePtr->tb < (uint16_t) HandlePtr->tmin)
  {
    HandlePtr->tb = (uint16_t) HandlePtr->tmin;
  }

  HandlePtr->v_ta = (uint16_t)(((uint32_t) HandlePtr->period + (uint16_t) HandlePtr->ta) + (uint16_t) HandlePtr->tb)
                     >> (uint16_t) 1;
  HandlePtr->v_tb = (uint16_t)((uint16_t) HandlePtr->period - ((uint16_t) HandlePtr->ta + (uint16_t) HandlePtr->tb))
                     >> (uint16_t) 1;

  /*Update the compare register with appropriate value  based on sector*/
  switch ((HandlePtr->sector & 7U))
    {
    case 0:/*sector 0*/
      HandlePtr->phaseu_crs = (uint32_t) HandlePtr->v_tb;
      HandlePtr->phasev_crs = (uint32_t) (HandlePtr->v_tb + HandlePtr->tb);
      HandlePtr->phasew_crs = (uint32_t) HandlePtr->v_ta;
       break;
    case 1:/*sector 1*/
      HandlePtr->phaseu_crs = (uint32_t) (HandlePtr->v_tb + HandlePtr->ta);
      HandlePtr->phasev_crs = (uint32_t) (HandlePtr->v_tb);
      HandlePtr->phasew_crs = (uint32_t) HandlePtr->v_ta;
       break;
    case 2:/*sector 2*/
      HandlePtr->phaseu_crs = (uint32_t) HandlePtr->v_ta;
      HandlePtr->phasev_crs = (uint32_t) HandlePtr->v_tb;
      HandlePtr->phasew_crs = (uint32_t) (HandlePtr->v_tb + HandlePtr->tb);
       break;
    case 3:/*sector 3*/
      HandlePtr->phaseu_crs = (uint32_t) HandlePtr->v_ta;
      HandlePtr->phasev_crs = (uint32_t)( HandlePtr->v_tb+ HandlePtr->ta);
      HandlePtr->phasew_crs = (uint32_t) HandlePtr->v_tb;
       break;
    case 4:/*sector 4*/
      HandlePtr->phaseu_crs = (uint32_t) HandlePtr->v_tb+ HandlePtr->tb;
      HandlePtr->phasev_crs = (uint32_t) HandlePtr->v_ta;
      HandlePtr->phasew_crs = (uint32_t) HandlePtr->v_tb;
       break;
    default:/*sector 5*/
      HandlePtr->phaseu_crs = (uint32_t) HandlePtr->v_tb;
      HandlePtr->phasev_crs = (uint32_t) HandlePtr->v_ta;
      HandlePtr->phasew_crs = (uint32_t) HandlePtr->v_tb+ HandlePtr->ta;
      break;
    }
}

