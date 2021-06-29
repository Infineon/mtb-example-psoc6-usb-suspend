/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the USB Suspend and Resume
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cy_usb_dev.h"
#include "cycfg_usbdev.h"


/***************************************************************************
* Constants
***************************************************************************/
/* Defines how often a message is printed in milliseconds */
#define MESSAGE_PRINT_PERIOD         (3000u)
/* Counter limit to track the suspend condition */
#define SUSPEND_COUNT                (3u)
/* USB COM port number */
#define USBUART_COM_PORT             (0U)
/* Timer period in millisecond */
#define TIMER_PERIOD_MS              (1u)
/* Timer frequency in Hz */
#define TIMER_FREQ_HZ                (1000000u)
/* Timer cycles to achieve the desired period */
#define TIMER_PERIOD_CYCLE           (TIMER_FREQ_HZ/TIMER_PERIOD_MS/1000)


/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void usb_high_isr(void);
static void usb_medium_isr(void);
static void usb_low_isr(void);
static void wakeup_pin_isr(void);
static void timer_callback(void *arg, cyhal_timer_event_t event);
static bool deepsleepcallback(cyhal_syspm_callback_state_t state,
        cyhal_syspm_callback_mode_t mode, void *callback_arg);


/*******************************************************************************
* Global Variables
********************************************************************************/
/* USB Interrupt Configuration */
const cy_stc_sysint_t usb_high_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_hi_IRQn,
    .intrPriority = 5U,
};
const cy_stc_sysint_t usb_medium_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_med_IRQn,
    .intrPriority = 6U,
};
const cy_stc_sysint_t usb_low_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_lo_IRQn,
    .intrPriority = 7U,
};
cy_stc_sysint_t wakeup_pin_interrupt_cfg =
{
    .intrSrc = ioss_interrupts_gpio_14_IRQn,
    .intrPriority = 0,
};

/* USBDEV context variables */
cy_stc_usbfs_dev_drv_context_t  usb_drvContext;
cy_stc_usb_dev_context_t        usb_devContext;
cy_stc_usb_dev_cdc_context_t    usb_cdcContext;

/* Initialize the timer object */
cyhal_timer_t timer_obj;

/* Global variables */
volatile bool     usb_suspend         = false;
volatile uint32_t usb_idle_counter    = 0u;
volatile uint32_t usb_msg_counter     = 0u;


/***************************************************************************
* Function Name: PrintMessage
********************************************************************************
* Summary:
*  Print a message to the COM terminal over USB CDC.
*
***************************************************************************/
void PrintMessage(const char msg[])
{
    if (Cy_USB_Dev_CDC_IsReady(USBUART_COM_PORT, &usb_cdcContext))
    {
        Cy_USB_Dev_CDC_PutData(USBUART_COM_PORT, (uint8_t *)msg, strlen(msg),
                                &usb_cdcContext);
    }
}


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU. It initializes the USB device block and
* enumerates as a CDC device. When the  USB suspend condition is detected, it 
* sends the device to a low power state, and restores normal operation when 
* USB activity resumes.
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

    cyhal_syspm_callback_data_t callback_data =
    {
        deepsleepcallback,
        CYHAL_SYSPM_CB_CPU_DEEPSLEEP,
        (cyhal_syspm_callback_mode_t) 0,
        NULL,
        NULL,
    };

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

   const cyhal_timer_cfg_t timer_cfg =
    {
        .compare_value = 0,
        .period = TIMER_PERIOD_CYCLE,
        .direction = CYHAL_TIMER_DIR_UP,
        .is_compare = false,
        .is_continuous = true,
        .value = 0
    };

    /* Initialize the timer */
    cyhal_timer_init(&timer_obj, NC, NULL);

    /* Power Management callback registration */
    cyhal_syspm_register_callback(&callback_data);

    /* Apply timer configuration */
    cyhal_timer_configure(&timer_obj, &timer_cfg);

    /* Set the timer frequency to 100000 */
    cyhal_timer_set_frequency(&timer_obj, TIMER_FREQ_HZ );

    /* Timer callback registration */
    cyhal_timer_register_callback(&timer_obj, timer_callback, NULL);

    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&timer_obj, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 3, true);

    /* Initialize the LED */
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_ON);

    /* Initialize the USB device */
    Cy_USB_Dev_Init(CYBSP_USBDEV_HW, &CYBSP_USBDEV_config, &usb_drvContext,
                        &usb_devices[0], &usb_devConfig, &usb_devContext);

    /* Initialize the CDC Class */
    Cy_USB_Dev_CDC_Init(&usb_cdcConfig, &usb_cdcContext, &usb_devContext);

    /* Initialize the USB interrupts */
    Cy_SysInt_Init(&usb_high_interrupt_cfg,   &usb_high_isr);
    Cy_SysInt_Init(&usb_medium_interrupt_cfg, &usb_medium_isr);
    Cy_SysInt_Init(&usb_low_interrupt_cfg,    &usb_low_isr);
    Cy_SysInt_Init(&wakeup_pin_interrupt_cfg,    &wakeup_pin_isr);

    /* Enable the USB interrupts */
    NVIC_EnableIRQ(usb_high_interrupt_cfg.intrSrc);
    NVIC_EnableIRQ(usb_medium_interrupt_cfg.intrSrc);
    NVIC_EnableIRQ(usb_low_interrupt_cfg.intrSrc);

    /* Setup USB Dp pin to generate an interrupt on falling edge */
    /* This interrupt wakes up the CPU from Deep-Sleep */
    Cy_GPIO_SetInterruptEdge(GPIO_PRT14, 0u, CY_GPIO_INTR_FALLING);
    Cy_GPIO_SetInterruptMask(GPIO_PRT14, 0u, 1UL);

    /* Make device appear on the bus. This function call is blocking,
     * it waits till the device enumerates
     */
    Cy_USB_Dev_Connect(true, CY_USB_DEV_WAIT_FOREVER, &usb_devContext);

    /* Start the timer */
    cyhal_timer_start(&timer_obj);

    /* Toggle the LED */
    cyhal_gpio_toggle(CYBSP_USER_LED);
    cyhal_system_delay_ms(250);
    cyhal_gpio_toggle(CYBSP_USER_LED);
    cyhal_system_delay_ms(250);
    cyhal_gpio_toggle(CYBSP_USER_LED);
    cyhal_system_delay_ms(250);
    cyhal_gpio_toggle(CYBSP_USER_LED);
    cyhal_system_delay_ms(250);

    /* Turn ON the LED */
    cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);

    for (;;)
    {
        /* Check if suspend condition is detected on bus */
        if (0u != usb_suspend)
        {
            /* Reset suspend detect variable */
            usb_suspend = 0;
            usb_idle_counter = 0u;

            /* Turn OFF the LED (Active Low) to indicate that the USBFS
             * is going to suspend mode
             */
            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);

            /* Enter deep-sleep mode */
            cyhal_syspm_deepsleep();

            /* Turn ON the LED to indicate that the USBFS is in active mode */
            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
        }

        /* Check if a message should be printed to the console */
        if (usb_msg_counter > MESSAGE_PRINT_PERIOD)
        {
             /* Reset message counter */
             usb_msg_counter = 0;

             /* Print message to the console */
             PrintMessage("USBFS is active\n\r");
         }

        /* Go to sleep */
        cyhal_syspm_sleep();
    }
}


/***************************************************************************
* Function Name: usb_high_isr
********************************************************************************
* Summary:
*  This function processes the high priority USB interrupts.
*
***************************************************************************/
static void usb_high_isr(void)
{
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(CYBSP_USBDEV_HW,
                               Cy_USBFS_Dev_Drv_GetInterruptCauseHi(CYBSP_USBDEV_HW),
                               &usb_drvContext);
}


/***************************************************************************
* Function Name: usb_medium_isr
********************************************************************************
* Summary:
*  This function processes the medium priority USB interrupts.
*
***************************************************************************/
static void usb_medium_isr(void)
{
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(CYBSP_USBDEV_HW,
                               Cy_USBFS_Dev_Drv_GetInterruptCauseMed(CYBSP_USBDEV_HW),
                               &usb_drvContext);
}


/***************************************************************************
* Function Name: usb_low_isr
********************************************************************************
* Summary:
*  This function processes the low priority USB interrupts.
*
**************************************************************************/
static void usb_low_isr(void)
{
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(CYBSP_USBDEV_HW,
                               Cy_USBFS_Dev_Drv_GetInterruptCauseLo(CYBSP_USBDEV_HW),
                               &usb_drvContext);
}


/***************************************************************************
* Function Name: timer_callback
********************************************************************************
* Summary:
*  One millisecond timer interrupt handler. Check for activity in the USB bus.
*
**************************************************************************/
void timer_callback(void *arg, cyhal_timer_event_t event)
{
    /* Check if there has been activity on USB bus since last timer tick */
    if (0u != Cy_USBFS_Dev_Drv_CheckActivity(CYBSP_USBDEV_HW))
    {
        usb_idle_counter = 0;
    }
    else
    {
        /* Check for suspend condition on USB bus */
        if (usb_idle_counter < SUSPEND_COUNT)
        {
            /* Counter idle time before detect suspend condition */
            usb_idle_counter++;

        }
        else
        {
            /* Suspend condition on USB bus is detected. Request device to
             * enter low-power mode
             */
            usb_suspend = true;
        }
    }

    /* Counter to print USB message */
    usb_msg_counter++;
}


/*******************************************************************************
* Function Name: wake_pin_isr
****************************************************************************//**
* Summary:
*  Wake-up pin interrupt handler for USB Dp. Clear the interrupt only.
*
*******************************************************************************/
void wakeup_pin_isr(void)
{
    /* Clear any pending interrupt */
    if (0u != Cy_GPIO_GetInterruptStatusMasked(GPIO_PRT14, 0u))
    {
        Cy_GPIO_ClearInterrupt(GPIO_PRT14, 0u);
    }
}


/*******************************************************************************
* Function Name: deepsleepcallback
********************************************************************************
* Summary:
*  Deep-sleep callback implementation.
*
* Parameters:
*  state - state the system or CPU is being transitioned into
*  mode  - callback mode
*  callback_arg   - user argument (not used)
*
* Return:
*  Always true
*
*******************************************************************************/
static bool deepsleepcallback(cyhal_syspm_callback_state_t state,
        cyhal_syspm_callback_mode_t mode, void *callback_arg)
{
    (void) callback_arg;

    switch(mode)
    {
        case CYHAL_SYSPM_CHECK_READY:

            /* Stop the timer */
            cyhal_timer_stop (&timer_obj);

            break;

        case CYHAL_SYSPM_BEFORE_TRANSITION:

            /* Prepare the device to move to deep sleep */
            Cy_USBFS_Dev_Drv_Suspend(CYBSP_USBDEV_HW, &usb_drvContext);

            /* Clear any pending interrupt */
            NVIC_ClearPendingIRQ(wakeup_pin_interrupt_cfg.intrSrc);

            /* Enable the Wake-up interrupt pin */
            NVIC_EnableIRQ(wakeup_pin_interrupt_cfg.intrSrc);

             break;

        case CY_SYSPM_AFTER_TRANSITION:

            /* Disable the Wake-up interrupt pin */
            NVIC_DisableIRQ(wakeup_pin_interrupt_cfg.intrSrc);

            /* Prepares the USBFS component for operation after exiting deep sleep mode */
            Cy_USBFS_Dev_Drv_Resume(CYBSP_USBDEV_HW, &usb_drvContext);

            /* Start the timer */
            cyhal_timer_start(&timer_obj);

            break;

        default:

            /* Failed to go to deep-sleep, start the timer */
            cyhal_timer_start(&timer_obj);

            break;

    }

    return true;
}
/* [] END OF FILE */
