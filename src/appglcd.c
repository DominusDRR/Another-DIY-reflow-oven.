/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    appglcd.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "appglcd.h"
#include "definitions.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APPGLCD_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APPGLCD_DATA appglcdData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************
extern uint32_t abs_diff_uint32(uint32_t a, uint32_t b);
extern bool IsGLCDTaskIdle (void);
extern void LCDSend(unsigned char data, unsigned char cd);
/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APPGLCD_Initialize ( void )

  Remarks:
    See prototype in appglcd.h.
 */

void APPGLCD_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appglcdData.state = APPGLCD_STATE_INIT;
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    appglcdData.adelay = RTC_Timer32CounterGet();
    LCD_RTS_Clear(); //Start resetting the GLCD module
}


/******************************************************************************
  Function:
    void APPGLCD_Tasks ( void )

  Remarks:
    See prototype in appglcd.h.
 */
void APPGLCD_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( appglcdData.state )
    {
        /* Application's initial state. */
        case APPGLCD_STATE_INIT:
        {
            if ( abs_diff_uint32(RTC_Timer32CounterGet(), appglcdData.adelay) > _1000ms)
            {
               appglcdData.adelay = RTC_Timer32CounterGet(); 
               LCD_RTS_Set(); 
               appglcdData.state = APPGLCD_STATE_WAIT_1000ms;    
            }    
            break;
        }
        case APPGLCD_STATE_WAIT_1000ms:
        {
            if ( abs_diff_uint32(RTC_Timer32CounterGet(), appglcdData.adelay) > _1000ms)
            {
                appglcdData.state = APPGLCD_STATE_LCD_EXTENDED_COMMANDS;
            }
            break;
        }
        /* TODO: implement your application state machine.*/
        case APPGLCD_STATE_LCD_EXTENDED_COMMANDS:
        {
            if (IsGLCDTaskIdle ()) //I wait until the SERCOM1 task is idle
            {
                LCDSend(0x21, SEND_CMD);
                appglcdData.state = APPGLCD_STATE_LCD_SET_VOP;
            }
            break;
        }
        case APPGLCD_STATE_LCD_SET_VOP:
        {
            if (IsGLCDTaskIdle ()) //I wait until the SERCOM1 task is idle
            {
                
            }
            break;
        }    
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}


/*******************************************************************************
 End of File
 */
