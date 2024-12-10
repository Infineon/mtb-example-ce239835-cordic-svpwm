/******************************************************************************
* File Name:   main.c
*
* Description: This code example demonstrates the 7-segment standard space vector
* modulation. Hardware CORDIC is used for sine and cosine calculation
* in SVPWM.This modulation technique is widely used in three-leg voltage inverter.
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

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "mtb_hal.h"
#include "cy_retarget_io.h"
#include "svpwm.h"

/*******************************************************************************
* Macros
********************************************************************************/

/*******************************************************************************
* Global Variables
********************************************************************************/
/* The PWM_U interrupt (Terminal count) configuration structure */
cy_stc_sysint_t tcpwm_intr_config =
{
    .intrSrc      = PWM_U_IRQ,
    .intrPriority = 1U
};

volatile uint16_t amplitude = 2200u;   /* Applied amplitude, Max amplitude  is 16384 (Q14)
                                          proportional to applied voltage*/
volatile uint32_t angle     = 0u;      /* Open loop angle*/
volatile uint32_t inc_angle = 40000u;  /* Incremental angle in every PWM cycle,
                                          proportional to speed */
volatile int16_t pot_value = 0;        /* Variable to store the potentiometer result */

/*SVPWM handler*/
SVPWM_t SVPWM_0;

/* For the Retarget -IO (Debug UART) usage */
static cy_stc_scb_uart_context_t    DEBUG_UART_context;           /** UART context */
static mtb_hal_uart_t               DEBUG_UART_hal_obj;           /** Debug UART HAL object  */

/*******************************************************************************
* Function Prototypes
********************************************************************************/
void tcpwm_intr_handler(void);
void TCPWM_PWMTimer_Init(void);
void TCPWM_PWMStartTimer_Init(void);
void Interrupt_Config(void);
/*******************************************************************************
* Function Definition
********************************************************************************/
void TCPWM_PWMTimer_Init(void)
{
    /*PWM phase U Initialization*/
    Cy_TCPWM_PWM_Init(PWM_U_HW, PWM_U_NUM, &PWM_U_config);
    Cy_TCPWM_PWM_Enable(PWM_U_HW, PWM_U_NUM);
    /*PWM phase V Initialization*/
    Cy_TCPWM_PWM_Init(PWM_V_HW, PWM_V_NUM, &PWM_V_config);
    Cy_TCPWM_PWM_Enable(PWM_V_HW, PWM_V_NUM);
    /*PWM phase W Initialization*/
    Cy_TCPWM_PWM_Init(PWM_W_HW, PWM_W_NUM, &PWM_W_config);
    Cy_TCPWM_PWM_Enable(PWM_W_HW, PWM_W_NUM);
}

/* PWM Start timer init*/
void TCPWM_PWMStartTimer_Init(void)
{
    Cy_TCPWM_PWM_Init(PWM_Start_HW, PWM_Start_NUM, &PWM_Start_config);
    Cy_TCPWM_PWM_Enable(PWM_Start_HW, PWM_Start_NUM);
}

/* Interrupt handler configuration */
void Interrupt_Config(void)
{
    NVIC_ClearPendingIRQ((IRQn_Type)tcpwm_intr_config.intrSrc);
    /* Sets up the interrupt handler */
    Cy_SysInt_Init(&tcpwm_intr_config, tcpwm_intr_handler);

    /* Enable PWM TC ISR */
    NVIC_EnableIRQ((IRQn_Type)tcpwm_intr_config.intrSrc);

}
/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Start the HPPASS autonomous controller (AC) from state 0*/
    if(CY_HPPASS_SUCCESS != Cy_HPPASS_AC_Start(0U, 1000U))
    {
        CY_ASSERT(0);
    }

    /* Debug UART init */
    result = (cy_rslt_t)Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);

    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);

    /* HAL UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);

    /* HAL retarget_io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    SVPWM_Init(&SVPWM_0);                  /*SVM parameter Init*/

    TCPWM_PWMTimer_Init();                 /* PWM Initialized */
    TCPWM_PWMStartTimer_Init();            /* This timer start the PWM in Sync */
    Cy_CORDIC_Enable(MXCORDIC);            /* Enable CORDIC module */
    Interrupt_Config();                    /* Interrupt Initialized */

    /* Enable global interrupts */
    __enable_irq();

    /* PWM Sync start */
    Cy_TCPWM_TriggerStart_Single(PWM_Start_HW, PWM_Start_NUM);

    printf("******************\r\n\n"
           "PWM Started, Monitor the modulated waveform on pins P4_0, P4_1, P4_2, P4_3, P4_4 and P4_5.\r\n\n"
           "Observe USER LED.\r\n\n"
           "Change potentiometer to vary LED toggling time.\r\n\n"
           "\r\n\n******************\r\n\n");

    for (;;)
    {
        /* Invert the USER LED state */
        Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);

        /* Read the potentiometer value, it will control the user LED toggling interval */
        pot_value    = Cy_HPPASS_SAR_Result_ChannelRead(CY_HPPASS_SAR_CHAN_12_IDX); //AN_B4

        /* Delay between LED toggles (control by pot value) */
        mtb_hal_system_delay_ms(pot_value);
    }
}

/*******************************************************************************
* Function Name: tcpwm_intr_handler
********************************************************************************
* Summary:
* This function is the TCPWM interrupt (PWM_U terminal count) handler . This ISR
* execute the SVM algorithm and update the duty cycle of Phase U, V and W timers.
* is executed from this ISR.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
/* PWM terminal count interrupt service routine */
void tcpwm_intr_handler(void)
{
    /*Clear Interrupt*/
    Cy_TCPWM_ClearInterrupt(PWM_U_HW, PWM_U_NUM, CY_TCPWM_INT_ON_TC);

    /*Generated the open loop angle for SVPWM module*/
    if (angle > 0xffffff)
    {
        angle = 0;
    }
    else
    {
        angle = angle + inc_angle;
        if(angle > 0xffffff)
        {
            angle = angle - 0xffffff;
        }
    }

    /*Call SVPWM with updated angle and amplitude*/
    SVPWM_SVMUpdate(&SVPWM_0,amplitude,angle);

    /* Update compare value to the TCPWM ,Compare register*/
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_U_HW, PWM_U_NUM, SVPWM_0.phaseu_crs);
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_V_HW, PWM_V_NUM, SVPWM_0.phasev_crs);
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_W_HW, PWM_W_NUM, SVPWM_0.phasew_crs);

    NVIC_ClearPendingIRQ((IRQn_Type)tcpwm_intr_config.intrSrc);
}



/* [] END OF FILE */
