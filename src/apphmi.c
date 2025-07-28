/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    apphmi.c

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

#include "apphmi.h"
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
    This structure should be initialized by the APPHMI_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APPHMI_DATA apphmiData;

const unsigned char *msgsHomeMenu[] = 
{
    (unsigned char *)" CHOOSE OPTION", // max 14 chars
    (unsigned char *)" ", 
    (unsigned char *)"Start process",
    (unsigned char *)"Set Parameters",
    (unsigned char *)"Current Temp"
};

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************
/* TODO:  Add any necessary callback functions.
*/
extern bool IsGLCDTaskIdle (void);
extern void LCDClear(void);
extern void LCDUpdate(void);
extern void LCDLine (int32_t x1, int32_t y1, int32_t x2, int32_t y2);
extern void LCDStr(uint8_t row, const uint8_t *dataPtr, bool inv, bool updateLCD);
extern void drawInitialLogo(void);
extern uint32_t abs_diff_uint32(uint32_t a, uint32_t b);
extern uint8_t getPressedBtn(void);
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
    void APPHMI_Initialize ( void )

  Remarks:
    See prototype in apphmi.h.
 */

void APPHMI_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    apphmiData.state = APPHMI_STATE_INIT;



    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APPHMI_Tasks ( void )

  Remarks:
    See prototype in apphmi.h.
 */

void APPHMI_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( apphmiData.state )
    {
        /* Application's initial state. */
        case APPHMI_STATE_INIT:
        {
            if (IsGLCDTaskIdle()) // I wait until the GLCD task is idle
            {
                drawInitialLogo();
                apphmiData.adelay = RTC_Timer32CounterGet();
                apphmiData.state = APPHMI_STATE_WAIT_DELAY_FOR_LOGO;
            }
            break;
        }
        case APPHMI_STATE_WAIT_DELAY_FOR_LOGO:
        {
            if ( abs_diff_uint32(RTC_Timer32CounterGet(), apphmiData.adelay) > _2000ms)
            {
                apphmiData.selectedOption = 0x02;
                apphmiData.state = APPHMI_STATE_CLEAR_LCD;
            }
            break;
        }
        case APPHMI_STATE_CLEAR_LCD:
        {
            if (IsGLCDTaskIdle()) // I wait until the GLCD task is idle
            {
                LCDClear();
                apphmiData.messagePointer = 0x00;
                apphmiData.state = APPHMI_STATE_DRAW_HOME_MENU;
            }
            break;
        }
        case APPHMI_STATE_DRAW_HOME_MENU:
        {
            if (IsGLCDTaskIdle()) // I wait until the GLCD task is idle
            {
                if (apphmiData.messagePointer < 0x05)
                {
                    LCDStr(apphmiData.messagePointer, msgsHomeMenu[apphmiData.messagePointer], (apphmiData.selectedOption == apphmiData.messagePointer), false);
                    apphmiData.messagePointer++;
                }
                else
                {
                    apphmiData.state = APPHMI_STATE_UPDATE_HOME_MENU;
                }
            }
            break;
        }
        /* TODO: implement your application state machine.*/
        case APPHMI_STATE_UPDATE_HOME_MENU:
        {
            if (IsGLCDTaskIdle()) // I wait until the GLCD task is idle
            {
                LCDUpdate();
                apphmiData.state = APPHMI_STATE_WAI_USER_SELECTION_HOME_MENU;
            }
            break;
        }
        case APPHMI_STATE_WAI_USER_SELECTION_HOME_MENU:
        {
            uint8_t button = getPressedBtn();
            if (NO_BUTTON_PRESSED != button)
            {
                switch (button)
                {
                    case OK_BUTTON_PRESSED:
                    {
                        
                        break;
                    }
                    case UP_BUTTON_PRESSED:
                    case DOWN_BUTTON_PRESSED:
                    {
                        if (button == UP_BUTTON_PRESSED)
                        {
                            apphmiData.selectedOption--;
                            if (apphmiData.selectedOption < 2)
                            {
                                apphmiData.selectedOption = 4;
                            }
                        }
                        else // DOWN
                        {
                            apphmiData.selectedOption++;
                            if (apphmiData.selectedOption > 4)
                            {
                                apphmiData.selectedOption = 2;
                            }
                        }
                        apphmiData.state = APPHMI_STATE_CLEAR_LCD;
                        break;
                    }
                    default: break; // case ESC
                }
            }
            break;
        }
        /* The default state should never be executed. */
        default: break; /* TODO: Handle error in application's state machine. */
    }
}


/*******************************************************************************
 End of File
 */
