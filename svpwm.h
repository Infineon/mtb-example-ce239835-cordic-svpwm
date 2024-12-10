/******************************************************************************
* File Name:   svpwm.h
*
* Description: 7-segment standard space vector modulation header file.
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

#ifndef SVPWM_H_
#define SVPWM_H_

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include <stdint.h>

/**********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/

/**
 * This is the macro for CORDIC MPS
 */
#define MPS  (1U)

/**
 * This is the macro for CORDIC inherent Gain Factor
 */
#define  K (1.646760258121f)

/**
 * Scale of MPS/K (to scale up MPS/K)
 */
#define MPS_K_SCALE_UP  (14U)

#define SVPWM_2_24  (16777216‬U)

/**
 * This is the macro for sin scale to eliminate gain factor K
 */
#define SVPWM_KSINSCALE          (9949U) //2^MPS_K_SCALE_UP*(MPS/K)
/**
 * This is the macro for cos scale to eliminate gain factor K
 */
#define SVPWM_KCOSSCALE          (17231U) //sqrt(3)* SVPWM_KSINSCALE
/**
 * This is the macro for sixty degree count
 */
#define SVPWM_SIXTYDEG_24BIT     (2796202U) //SVPWM_2_24/6

/**********************************************************************************************************************
* ENUMS
**********************************************************************************************************************/


/**********************************************************************************************************************
* DATA STRUCTURES
**********************************************************************************************************************/
/**
 * This structure holds the general APP data.
 */
typedef struct SVPWM
{
  volatile  uint32_t                phaseu_crs;
  volatile  uint32_t                phasev_crs;
  volatile  uint32_t                phasew_crs;

  uint32_t                           deadtime_rising_edge;            /*!< This variable represents dead time for rising edge*/
  uint32_t                           deadtime_falling_edge;           /*!< This variable represents dead time for falling edge*/
  uint32_t                           period;                          /*!< This variable represents period value*/
  uint32_t                           sector;                          /*!< Sector value 0-5*/
  uint32_t                           sector_angle;                    /*!<Sector angle, 60 degree is represented as 1023*/
  uint32_t                           period_max;                       /*!< Max period value*/
  uint32_t                           period_min;                       /*!< Min period value*/
  uint32_t                           pwm_frequency;                    /*!< This represents PWM frequency value.*/

  uint16_t                           t0;                              /*!< zero vector segment time*/
  uint16_t                           ta;                              /*!< Ta Segment Time calculation variable*/
  uint16_t                           tb;                              /*!< Tb Segment Time calculation variable*/
  uint16_t                           v_ta;                            /*!< Compare Value calculation  Variable */
  uint16_t                           v_tb;                            /*!< Compare Value calculation  Variable*/
  uint16_t                           tmin;                            /*!< This variable represents tmin count*/
  uint16_t                           max_amplitude;                   /*!< This variable represents maximum amplitude calculation*/
  uint16_t                           amplitude_scale;

}SVPWM_t;


extern SVPWM_t SVPWM_0;

/***********************************************************************************************************************
 * API Prototypes
 **********************************************************************************************************************/
void  SVPWM_Init(SVPWM_t* const HandlePtr);

/**
 * @brief Executes the SVM algorithm.
 * @param HandlePtr Handle of the SVPWM APP
 * @param Amplitude Voltage to be applied to the motor.
 * 0 - 100% => 0- PWM Period Value
 * @param Angle Angle of the rotor, 0 -360 degrees => 0 - 0xFFFFFF.
 * @return None<BR>
 */
void SVPWM_SVMUpdate(SVPWM_t* const HandlePtr, uint16_t Amplitude, uint32_t Angle);


#endif //End of SVPWM_H_
