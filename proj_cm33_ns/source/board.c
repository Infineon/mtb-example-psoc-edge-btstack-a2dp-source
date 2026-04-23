/*******************************************************************************
* File Name: board.c
*
* Description: This file contains board supported API's.
*
* Related Document: See README.md
*
*******************************************************************************
* (c) 2024-2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

/*******************************************************************************
* Header file includes
*******************************************************************************/
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <FreeRTOS.h>
#include <task.h>
#include "board.h"
#include "app_bt_utils.h"
#include "retarget_io_init.h"
#include "mtb_syspm_callbacks.h"
/*******************************************************************************
* Macros
********************************************************************************/
#define PWM_DUTY_CYCLE_0                (0U)
#define PWM_DUTY_CYCLE_50               (1000U)
#define PWM_DUTY_CYCLE_100              (2000U)
#define GPIO_INTERRUPT_PRIORITY         (7U)
#define PORT_INTR_MASK                  (0x00000001UL << 8U)
#define BTN_DEBOUNCE_DELAY              (300U)
#define INTERRUPT_MASKED                (1UL)

/*******************************************************************************
* Global Variables
*******************************************************************************/
extern bool button_status_flag ;

#if (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP)

/* PWM Context for SysPm Callback */
mtb_syspm_tcpwm_deepsleep_context_t  PWM1dscontext;

/* SysPm callback parameter structure for PWM_1 */
static cy_stc_syspm_callback_params_t PWM1DSParams =
{
    .context   = &PWM1dscontext,
    .base      = CYBSP_PWM_LED_CTRL_HW
};


/* SysPm callback structure for PWM_1 */
static cy_stc_syspm_callback_t PWM1DeepSleepCallbackHandler =
{
    .callback           = &mtb_syspm_tcpwm_deepsleep_callback,
    .skipMode           = CY_SYSPM_SKIP_CHECK_FAIL | CY_SYSPM_SKIP_CHECK_READY,
    .type               = CY_SYSPM_DEEPSLEEP,
    .callbackParams     = &PWM1DSParams,
    .prevItm            = NULL,
    .nextItm            = NULL,
    .order              = SYSPM_CALLBACK_ORDER
};


#endif /*CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP) */
/*******************************************************************************
* Function Definition
*******************************************************************************/

/*******************************************************************************
* Function Name: button_interrupt_callback
********************************************************************************
* Summary:
*  GPIO interrupt handler.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void button_interrupt_callback(void)
{
    /* Get interrupt cause */
    uint32_t intrSrc = Cy_GPIO_GetInterruptCause0();

    /* User Button Debounce Delay */
    Cy_SysLib_Delay(BTN_DEBOUNCE_DELAY);

    /* Check if the interrupt was from the user button's port */
    if(PORT_INTR_MASK == (intrSrc & PORT_INTR_MASK))
    {
        if(INTERRUPT_MASKED == Cy_GPIO_GetInterruptStatusMasked(CYBSP_USER_BTN_PORT,
                                                                   CYBSP_USER_BTN_PIN))
        {
            /* Clear the interrupt */
            Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN);
            NVIC_ClearPendingIRQ(CYBSP_USER_BTN1_IRQ);

            button_status_flag = true;


        }
    }
}

/*******************************************************************************
* Function Name: board_button_init
********************************************************************************
*
* Summary:
*  Initialize the button with Interrupt
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void board_button_init(void)
{
    /* Interrupt config structure */
    cy_stc_sysint_t intrCfg =
    {
        .intrSrc = CYBSP_USER_BTN_IRQ,
        .intrPriority = GPIO_INTERRUPT_PRIORITY
    };

    /*Clear GPIO and NVIC interrupt before initializing to avoid false
     * triggering.
     */
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN_PORT,CYBSP_USER_BTN_PIN);
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN_PORT,CYBSP_USER_BTN2_PIN);
    NVIC_ClearPendingIRQ(CYBSP_USER_BTN1_IRQ);

    /* Initialize the interrupt and register interrupt callback */
    Cy_SysInt_Init(&intrCfg, &button_interrupt_callback);

    /* Enable the interrupt in the NVIC */
    NVIC_EnableIRQ(intrCfg.intrSrc);
}

/*******************************************************************************
* Function Name: board_led_init
********************************************************************************
*
* Summary:
*   Initialize the User Led 3 with PWM
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
static void board_led_init(void)
{
    cy_rslt_t result = CY_TCPWM_SUCCESS;

    /* Initialize the TCPWM block */
    result = Cy_TCPWM_PWM_Init(CYBSP_PWM_LED_CTRL_HW,
            CYBSP_PWM_LED_CTRL_NUM,
            &CYBSP_PWM_LED_CTRL_config);

    /* PWM init failed. Stop program execution */
    if(CY_TCPWM_SUCCESS != result)
    {
        printf("Failed to initialize PWM for LED.\r\n");
        CY_ASSERT(0U);
    }

    /* Enable the TCPWM block */
    Cy_TCPWM_PWM_Enable(CYBSP_PWM_LED_CTRL_HW,
            CYBSP_PWM_LED_CTRL_NUM);
            
    PWM1dscontext.channelNum = CYBSP_PWM_LED_CTRL_NUM;

#if (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP)

    /* SysPm callback registration for PWMs */
    Cy_SysPm_RegisterCallback(&PWM1DeepSleepCallbackHandler);

#endif  

    /* Start the PWM */
    Cy_TCPWM_TriggerStart_Single(CYBSP_PWM_LED_CTRL_HW,
            CYBSP_PWM_LED_CTRL_NUM);

    /* Turn LED off during initialization */
    Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_LED_CTRL_HW,
              CYBSP_PWM_LED_CTRL_NUM,PWM_DUTY_CYCLE_0);


}

/*******************************************************************************
* Function Name: board_init
********************************************************************************
*
* Summary:
*  Initialize the board with LED's and Buttons
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void board_init(void)
{
    /* Initialize the USER LEDs with PWM*/
    board_led_init();

    /* Initialize the USER BUTTON */
    board_button_init();
}

/*******************************************************************************
* Function Name: board_led_set_state
********************************************************************************
*
* Summary:
*  Set the led state over PWM
*
* Parameters:
*  value: ON/OFF state
*
* Return:
*  None
*
*******************************************************************************/
void board_led_set_state(bool value)
{
        Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_LED_CTRL_HW,
                CYBSP_PWM_LED_CTRL_NUM,value?PWM_DUTY_CYCLE_0:PWM_DUTY_CYCLE_100);
}

/*******************************************************************************
* Function Name: board_led_set_blink
********************************************************************************
*
* Summary:
*  Set the User led 3 duty cycle to 50%
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void board_led_set_blink(void)
{
    Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_LED_CTRL_HW,
            CYBSP_PWM_LED_CTRL_NUM,PWM_DUTY_CYCLE_50);
}


/* [] END OF FILE */
