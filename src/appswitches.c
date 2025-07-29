/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    appswitches.c

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

#include "appswitches.h"
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
    This structure should be initialized by the APPSWITCHES_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APPSWITCHES_DATA appswitchesData;

typedef bool (*ButtonReadFn)(void);

typedef struct 
{
    ButtonReadFn    read;
    uint8_t         code;
} ButtonMap_t;

static inline bool OK_Read(void)   { return (OK_F_Get()   == 0) ? false : true; }
static inline bool ESC_Read(void)  { return (ESC_F_Get()  == 0) ? false : true; }
static inline bool UP_Read(void)   { return (UP_F_Get()   == 0) ? false : true; }
static inline bool DOWN_Read(void) { return (DOWN_F_Get() == 0) ? false : true; }

static const ButtonMap_t buttonMap[] = 
{
    { OK_Read,    OK_BUTTON_PRESSED   },
    { ESC_Read,   ESC_BUTTON_PRESSED  },
    { UP_Read,    UP_BUTTON_PRESSED   },
    { DOWN_Read,  DOWN_BUTTON_PRESSED }
};

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************
/* TODO:  Add any necessary callback functions.
*/
extern uint32_t abs_diff_uint32(uint32_t a, uint32_t b);
extern void buttonPressedSound(void);
// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************
uint8_t getPressedBtn(void);
void resetButtonScan(void);
/* TODO:  Add any necessary local functions.
*/
uint8_t getPressedBtn(void)
{
    if (APPSWITCHES_STATE_WAIT_BUTTON_READING == appswitchesData.state)
    {
        uint8_t button = appswitchesData.buttonPressed;
        resetButtonScan();
        return button;//appswitchesData.buttonPressed;
    }
    return NO_BUTTON_PRESSED;
}
void resetButtonScan(void)
{
    appswitchesData.buttonPressed = NO_BUTTON_PRESSED;
    appswitchesData.buttonPointer = 0x00;
    appswitchesData.state = APPSWITCHES_STATE_CHECK_BUTTONS;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APPSWITCHES_Initialize ( void )

  Remarks:
    See prototype in appswitches.h.
 */

void APPSWITCHES_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appswitchesData.state = APPSWITCHES_STATE_INIT;
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    appswitchesData.adelay = RTC_Timer32CounterGet();
    appswitchesData.buttonPressed = NO_BUTTON_PRESSED;
    appswitchesData.buttonPointer = 0x00;
}


/******************************************************************************
  Function:
    void APPSWITCHES_Tasks ( void )

  Remarks:
    See prototype in appswitches.h.
 */

void APPSWITCHES_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appswitchesData.state )
    {
        /* Application's initial state. */
        case APPSWITCHES_STATE_INIT:
        {
            if ( abs_diff_uint32(RTC_Timer32CounterGet(), appswitchesData.adelay) > _200ms) //I wait until the hardware is stable
            {
                appswitchesData.state = APPSWITCHES_STATE_CHECK_BUTTONS;
            }
            break;
        }
        case APPSWITCHES_STATE_CHECK_BUTTONS:
        {
            if (appswitchesData.buttonPointer < 0x04)
            {
                if (!buttonMap[appswitchesData.buttonPointer].read()) 
                {
                    appswitchesData.buttonPressed = buttonMap[appswitchesData.buttonPointer].code;
                    appswitchesData.adelay = RTC_Timer32CounterGet();
                    buttonPressedSound();
                    appswitchesData.state = APPSWITCHES_STATE_ELIMINATE_BOUNCE;
                    return;
                }
                appswitchesData.buttonPointer++;
            }
            else
            {
                appswitchesData.buttonPointer = 0x00;    
            }
            break;
        }
        /* TODO: implement your application state machine.*/
        case APPSWITCHES_STATE_ELIMINATE_BOUNCE:
        {
            if ( abs_diff_uint32(RTC_Timer32CounterGet(), appswitchesData.adelay) > _200ms) //I wait until I eliminate the bounce
            {
                appswitchesData.state = APPSWITCHES_STATE_WAIT_UNTIL_BUTTON_RELEASED;
            }
            break;
        }
        case APPSWITCHES_STATE_WAIT_UNTIL_BUTTON_RELEASED:
        {
            if ( buttonMap[appswitchesData.buttonPointer].read()) 
            {
                appswitchesData.adelay = RTC_Timer32CounterGet();
                appswitchesData.state = APPSWITCHES_STATE_WAIT_BUTTON_READING;
            }
            break;
        }
        case APPSWITCHES_STATE_WAIT_BUTTON_READING:
        {
            if ( abs_diff_uint32(RTC_Timer32CounterGet(), appswitchesData.adelay) > _200ms) //I wait an additional 200 ms until another task reads the button that has been pressed
            {
                resetButtonScan();
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
