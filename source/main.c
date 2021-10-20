/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for MQTT Client Example for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
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

/* Header file includes */
#include "feature_config.h"
#include "cy_debug.h"

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "mqtt_task.h"

#if (FEATURE_ABSTRACTION_RTOS == ENABLE_FEATURE)
#include "cyabs_rtos.h"

#else
    #ifdef COMPONENT_FREERTOS
    #include "FreeRTOS.h"
    #include "task.h"
    #endif
#endif

/******************************************************************************
* Global Variables
******************************************************************************/
/* This enables RTOS aware debugging. */
volatile int uxTopUsedPriority;

/******************************************************************************
 * Function Name: main
 ******************************************************************************
 * Summary:
 *  System entrance point. This function initializes retarget IO, sets up 
 *  the MQTT client task, and then starts the RTOS scheduler.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 ******************************************************************************/
int main_thread(void)
{
    cy_rslt_t result;

    /* Enable global interrupts. */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port. */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                        CY_RETARGET_IO_BAUDRATE);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen. */
    printf("\x1b[2J\x1b[;H");
    printf("===============================================================\n");
    printf("CE229889 - AnyCloud Example: MQTT Client\n");
    printf("===============================================================\n\n");

#if (FEATURE_ABSTRACTION_RTOS == ENABLE_FEATURE)
    result = cy_rtos_create_thread( &mqtt_task_handle,
                                    mqtt_client_task,
                                    "MQTT Client task",
                                    NULL,
                                    MQTT_CLIENT_TASK_STACK_SIZE,
                                    MQTT_CLIENT_TASK_PRIORITY,
                                    (cy_thread_arg_t) NULL
                                    );

    if (CY_RSLT_SUCCESS != result) {
        DEBUG_PRINT(("cy_rtos_create_thread (mqttThread) failed: (0x%lx)",
                         result
                        ));
    }

#else
    /* Create the MQTT Client task. */
    xTaskCreate(mqtt_client_task, "MQTT Client task", MQTT_CLIENT_TASK_STACK_SIZE, 
                NULL, MQTT_CLIENT_TASK_PRIORITY, NULL);
#endif

    return 0;
}

int main(void)
{
#ifdef COMPONENT_FREERTOS
    int result;

    uxTopUsedPriority = configMAX_PRIORITIES - 1;

    if (CY_RSLT_SUCCESS != cybsp_init()) {
        CY_ASSERT(0);
    }

    result = main_thread();

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never get here */
    CY_ASSERT(0);

    return result;

#elif defined COMPONENT_RTTHREAD
    extern int entry(void);

    static bool initialized = false;

    uxTopUsedPriority = RT_THREAD_PRIORITY_MAX - 1 ;

    if (!initialized) {
        initialized = true;
        return entry();
    }
    else {
        return main_thread();
    }
#endif
}

/* [] END OF FILE */
