/*******************************************************************************
 * File Name: main.c
 *
 * Description: This is the source code for the  Bluetooth Classic A2DP source
 *              Example for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *******************************************************************************
 * Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company) or
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

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "cy_pdl.h"
#include "cycfg.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "app_bt_utils/app_bt_utils.h"
#include "a2dp_source.h"
#include "app_bt_bonding.h"
#include "app_bt_utils.h"
#include "retarget_io_init.h"
#include "board.h"
#include "cy_time.h"
/******************************************************************************
 * Macros
 *****************************************************************************/
/* The timeout value in microsecond used to wait for core to be booted */
#define CM55_BOOT_WAIT_TIME_US            (10U)

/* App boot address for CM55 project */
#define CM55_APP_BOOT_ADDR          (CYMEM_CM33_0_m55_nvm_START + \
                                        CYBSP_MCUBOOT_HEADER_SIZE)

/* UART function parameter value to wait forever */
#define HEX_BASE                          (16U)
#define BT_STACK_HEAP_SIZE                (0X1000U)
#define BT_TASK_STACK_SIZE                (1024U)
#define BT_TASK_PRIORITY                  (configMAX_PRIORITIES - (6U))
#define TASK_DELAY_50MS                   (50U)
#define TIMER_INTERRUPT_PRIORITY          (7U)
#define INIT_VALUE                        (0U)

/* Enabling or disabling a MCWDT requires a wait time of upto 2 CLK_LF cycles
 * to come into effect. This wait time value will depend on the actual CLK_LF
 * frequency set by the BSP.
 */
#define LPTIMER_0_WAIT_TIME_USEC            (62U)

/* Define the LPTimer interrupt priority number. '1' implies highest priority.
 */
#define APP_LPTIMER_INTERRUPT_PRIORITY      (1U)

/****************************************************************************
 * Global variables
 ***************************************************************************/
wiced_bt_device_address_t bda_str;
uint8_t uart_response;

/****************************************************************************
 * Static and extern variables
 ***************************************************************************/
static TaskHandle_t bt_task_handle;
/* LPTimer HAL object */
static mtb_hal_lptimer_t lptimer_obj;

/* RTC HAL object */
static mtb_hal_rtc_t rtc_obj;

extern TaskHandle_t a2dp_task_handle;
extern device_info_t device_info_list[MAX_DEVICES];
extern int device_count;
/*****************************************************************************
 * Function Prototypes
 *****************************************************************************/
static void application_start(void *task_params);
static void bt_task_create(void);

/*****************************************************************************
 * Function Definitions
 *****************************************************************************/
/*******************************************************************************
* Function Name: lptimer_interrupt_handler
********************************************************************************
* Summary:
* Interrupt handler function for LPTimer instance.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void lptimer_interrupt_handler(void)
{
    mtb_hal_lptimer_process_interrupt(&lptimer_obj);
}

/*******************************************************************************
* Function Name: setup_tickless_idle_timer
********************************************************************************
* Summary:
* 1. This function first configures and initializes an interrupt for LPTimer.
* 2. Then it initializes the LPTimer HAL object to be used in the RTOS
*    tickless idle mode implementation to allow the device enter deep sleep
*    when idle task runs. LPTIMER_0 instance is configured for CM33 CPU.
* 3. It then passes the LPTimer object to abstraction RTOS library that
*    implements tickless idle mode
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void setup_tickless_idle_timer(void)
{
    /* Interrupt configuration structure for LPTimer */
    cy_stc_sysint_t lptimer_intr_cfg =
    {
        .intrSrc = CYBSP_CM33_LPTIMER_0_IRQ,
        .intrPriority = APP_LPTIMER_INTERRUPT_PRIORITY
    };

    /* Initialize the LPTimer interrupt and specify the interrupt handler. */
    cy_en_sysint_status_t interrupt_init_status =
                                    Cy_SysInt_Init(&lptimer_intr_cfg,
                                                    lptimer_interrupt_handler);

    /* LPTimer interrupt initialization failed. Stop program execution. */
    if(CY_SYSINT_SUCCESS != interrupt_init_status)
    {
        handle_app_error();
    }

    /* Enable NVIC interrupt. */
    NVIC_EnableIRQ(lptimer_intr_cfg.intrSrc);

    /* Initialize the MCWDT block */
    cy_en_mcwdt_status_t mcwdt_init_status =
                                    Cy_MCWDT_Init(CYBSP_CM33_LPTIMER_0_HW,
                                                &CYBSP_CM33_LPTIMER_0_config);

    /* MCWDT initialization failed. Stop program execution. */
    if(CY_MCWDT_SUCCESS != mcwdt_init_status)
    {
        handle_app_error();
    }

    /* Enable MCWDT instance */
    Cy_MCWDT_Enable(CYBSP_CM33_LPTIMER_0_HW,
                    CY_MCWDT_CTR_Msk,
                    LPTIMER_0_WAIT_TIME_USEC);

    /* Setup LPTimer using the HAL object and desired configuration as defined
     * in the device configurator. */
    cy_rslt_t result = mtb_hal_lptimer_setup(&lptimer_obj,
                                            &CYBSP_CM33_LPTIMER_0_hal_config);

    /* LPTimer setup failed. Stop program execution. */
    if(CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    /* Pass the LPTimer object to abstraction RTOS library that implements
     * tickless idle mode
     */
    cyabs_rtos_set_lptimer(&lptimer_obj);
}

/*******************************************************************************
* Function Name: setup_clib_support
********************************************************************************
* Summary:
*    1. This function configures and initializes the Real-Time Clock (RTC).
*    2. It then initializes the RTC HAL object to enable CLIB support library
*       to work with the provided Real-Time Clock (RTC) module.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void setup_clib_support(void)
{
    /* RTC Initialization */
    Cy_RTC_Init(&CYBSP_RTC_config);
    Cy_RTC_SetDateAndTime(&CYBSP_RTC_config);

    /* Initialize the ModusToolbox CLIB support library */
    mtb_clib_support_init(&rtc_obj);
}

/*****************************************************************************
 * Function Name: main()
 ******************************************************************************
 * Summary:
 *   Entry point to the application.
 *   Set device configuration and start BT application RTOS tasks
 *   Enables CM55
 *
 * Parameters:
 *          None
 *
 * Return:
 *          result status
 *
 ******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize and Verify the BSP initialization */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    /* Setup CLIB support library. */
    setup_clib_support();

    /* Setup the LPTimer instance for CM33 CPU. */
    setup_tickless_idle_timer();

    /* Initialize retarget-io middleware */
    init_retarget_io();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen */
    printf("\x1b[2J\x1b[;H");
    printf( "\r\n********************************************************" );
    printf( "\r\n        PSOC EDGE MCU: A2DP Source + AVRCP          \r\n" );
    printf( "********************************************************\r\n" );

    /* Initialize USER Button and USER LED */
    board_init();

    /* Initialize the device used by kv-store for performing
     * read/write operations to the NVM
     */
    app_kv_store_init();

    if(BUTTON_PRESSED == Cy_GPIO_Read(CYBSP_USER_BTN1_PORT, CYBSP_USER_BTN1_PIN))
    {
         /* Reset Kv-store library, this will clear the keys in the nvm */
         reset_link_keys();
         handle_app_error();
    }

    /* Creating BT task */
    bt_task_create();

    /* Enable global interrupts */
    __enable_irq();

    /* Enable CM55. CY_CORTEX_M55_APPL_ADDR must be updated if CM55 memory layout is changed. */
    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_US);

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never get here */
    handle_app_error();
}

/******************************************************************************
 * Function Name: uart_get_data()
 *******************************************************************************
 * Summary:
 *         This function reads a single byte from the UART interface, 
 *         blocking until data is available.
 *
 * Parameters:
 *          value : uart data
 *
 * Return:
 *          None
 *
 ******************************************************************************/
static void uart_get_data(uint8_t *value)
{

    uint32_t read_value = Cy_SCB_UART_Get(SCB2);
    while (CY_SCB_UART_RX_NO_DATA == read_value)
    {
        read_value = Cy_SCB_UART_Get(SCB2);
        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_10MS));
    }
    *value = (uint8_t)read_value;
}

/******************************************************************************
 * Function Name: print_stored_devices(void)
 *******************************************************************************
 * Summary:
 *          Prints the Stored BT devices after scanning
 *
 ******************************************************************************/
static void print_stored_devices(void)
{
    for (int i = 0; i < device_count; i++)
    {
        printf("Device %d: ", i + 1);
        printf("BD_ADDR: ");
        for (int j = 0; j < BD_ADDR_LEN; j++)
        {
            if(j != (BD_ADDR_LEN -1))
                printf("%02X:", device_info_list[i].remote_bd_addr[j]);
            else
                printf("%02X", device_info_list[i].remote_bd_addr[j]);
        }
        printf("\tName: %s", device_info_list[i].eir_data);
        printf("\n");
    }
}

/******************************************************************************
 * Function Name: application_start()
 *******************************************************************************
 * Summary:
 *          Starts BT stack and runs the BT thread monitoring for connection status.
 *
 * Parameters:
 *          Task parameters.
 *
 * Return:
 *          None
 *
 ******************************************************************************/
static void application_start(void *task_params)
{
    wiced_result_t wiced_result = WICED_BT_SUCCESS;
    cy_rslt_t result;


    /* Switch OFF the LED */
    board_led_set_state(LED_OFF);

    /* A2DP Sorce audio thread */
    result = xTaskCreate(a2dp_source_audio_thread, "Streaming task", A2DP_TASK_STACK_SIZE, NULL, A2DP_TASK_PRIORITY, &a2dp_task_handle);

    if (pdPASS != result)
    {
        printf("Error in creating Streaming task \n");
    }

    /* Register call back and configuration with stack */
    wiced_result = wiced_bt_stack_init(a2dp_source_management_callback, &a2dp_source_cfg_settings);

    WICED_BT_TRACE("Initializing BT Stack - A2DP Source Start\n");

    /* Check if stack initialization was successful */
    if (WICED_BT_SUCCESS == wiced_result)
    {
        /* Create a buffer heap, make it the default heap.  */
        p_default_heap = wiced_bt_create_heap("app", NULL, BT_STACK_HEAP_SIZE,
                NULL, WICED_TRUE);
        WICED_BT_TRACE("Creating heap \r\n");

        if ((WICED_BT_SUCCESS == wiced_result) && (NULL != p_default_heap))
        {
            WICED_BT_TRACE("heap creation successful\r\\n");
            fprintf(stdout, "Loading Bluetooth Stack...\r");
        }
        else /* Exit App if stack init was not successful or heap creation failed */
        {
            fprintf(stderr, "Bluetooth Stack Initialization or heap creation failed!! Exiting App...\n\r");
            CY_ASSERT(0);
        }
    }
    printf("Loading Bluetooth Stack & tasks...\n\r");

    bda_str[0]=BDA_ADDRESS_BYTE_0;
    bda_str[1]=BDA_ADDRESS_BYTE_1;
    bda_str[2]=BDA_ADDRESS_BYTE_2;
    bda_str[3]=BDA_ADDRESS_BYTE_3;
    bda_str[4]=BDA_ADDRESS_BYTE_4;
    bda_str[5]=BDA_ADDRESS_BYTE_5;

    for (;;)
    {
        if (initiate_bt_device_connection )
        {
            if (device_count > 0)
            {
                print_stored_devices();

                printf("Enter the device number you wish to connect to:\n");

                uart_get_data(&uart_response);

                if (uart_response > '0')
                {
                    int idx = uart_response - '1';

                    if (idx <= device_count)
                    {

                        printf("Connecting to: %s \r\n", device_info_list[idx].eir_data);

                        wiced_result = a2dp_source_bt_set_visibility(WICED_TRUE, WICED_TRUE);

                        if (WICED_BT_SUCCESS == wiced_result)
                        {
                            wiced_result = a2dp_source_command_connect(device_info_list[idx].remote_bd_addr, BD_ADDR_LEN);

                            if (WICED_BT_SUCCESS == wiced_result)
                            {
                                printf("Connection Request sent. Waiting for A2DP sink device to get connected...\r\n");
                                initiate_bt_device_connection = false;
                            }
                            else
                            {
                                printf("Error: Failed to connect \r\n");
                            }
                        }
                        else
                        {
                            printf("Error: Failed to set the Bluetooth visibility \r\n");
                        }
                    }
                    else{
                        printf("Invalid device number. Initiating re-scan...\r\n");
                        device_count = 0;
                        initiate_bt_device_connection = false;
                        a2dp_source_bt_set_pairability(true);
                        a2dp_source_bt_inquiry (true);
                    }

                }
            }

            else
            {
                printf("No Bluetooth sink devices found. Would you like to retry? (y/n): \r\n");

                uart_get_data(&uart_response);

                if (uart_response == 'y' ||  uart_response == 'Y')
                {
                    printf("Re-scanning..\r\n");
                    device_count = 0;
                    initiate_bt_device_connection = false;
                    a2dp_source_bt_set_pairability(true);
                    a2dp_source_bt_inquiry (true);
                }
                else
                {
                    printf("Scan cancelled...\r\n");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_50MS));

        if (A2DP_PEER_CONNECTED == connection_status)
        {
            board_led_set_state(LED_ON);
        }
        else if (A2DP_PEER_DISCONNECTED == connection_status)
        {
            board_led_set_blink();
        }
    }
}

/******************************************************************************
 * Function Name: bt_task_create()
 *******************************************************************************
 * Summary:
 *           BT task creation function wrapper
 *
 * Parameters:
 *           None
 *
 * Return:
 *          None
 *
 ******************************************************************************/
static void bt_task_create(void)
{
    BaseType_t status;
    status = xTaskCreate(application_start, "BT task", BT_TASK_STACK_SIZE,
            NULL, BT_TASK_PRIORITY, &bt_task_handle);
    if (pdPASS != status)
    {
        printf("Error in starting BT task \n");
    }
}


/* [] END OF FILE */
