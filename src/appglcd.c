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
#define LCD_START_LINE_ADDR	(66-2)
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
const uint8_t commandsInitializeGLCD[0x08] = 
{
    0x21, // 0 LCD Extended Commands.
    0xC8, // 1 Set LCD Vop (Contrast). 0xC8
    0x04 | !!(LCD_START_LINE_ADDR & (1u << 6)),  // 2 Set Temp S6 for start line
    0x40 | (LCD_START_LINE_ADDR & ((1u << 6) - 1)), // 3 Set Temp S[5:0] for start line
    0x12, // 4 LCD bias mode 1:68.
    0x20, // 5 LCD Standard Commands, Horizontal addressing mode.
    0x08, // 6 LCD blank
    0x0C  // 7 LCD in normal mode
};
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************
extern uint32_t abs_diff_uint32(uint32_t a, uint32_t b);
extern bool IsSPI1TaskIdle (void);
extern void LCDSend(unsigned char data, unsigned char cd);
/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

bool IsGLCDTaskIdle (void)
{
    if (APPGLCD_STATE_IDLE == appglcdData.state)
    {
        return true;
    }
    return false;
}

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
                appglcdData.pointerX = 0x00;
                appglcdData.state = APPGLCD_STATE_LCD_COMMANDS;
            }
            break;
        }
        /* TODO: implement your application state machine.*/
        case APPGLCD_STATE_LCD_COMMANDS:
        {
            if (IsSPI1TaskIdle ()) //I wait until the SERCOM1 task is idle
            {
                LCDSend(commandsInitializeGLCD[appglcdData.pointerX], SEND_CMD);
                appglcdData.pointerX++;
                if (appglcdData.pointerX >= sizeof(commandsInitializeGLCD))
                {    
                    appglcdData.state = APPGLCD_STATE_START_CLEANING_GLCD;
                }    
            }
            break;
        }
        case APPGLCD_STATE_START_CLEANING_GLCD:
        {
            if (IsSPI1TaskIdle ()) //I wait until the SERCOM1 task is idle
            {
                appglcdData.pointerX = 0x00;
                appglcdData.state = APPGLCD_STATE_CLEAR_LCD;
            }
            break;
        }
        case APPGLCD_STATE_CLEAR_LCD:
        {
            if (appglcdData.pointerX < sizeof(appglcdData.LcdMemory))
            {
                appglcdData.LcdMemory[appglcdData.pointerX] = 0x00;
                appglcdData.pointerX++;
            }
            else
            {
                appglcdData.pointerY = 0x00;
                appglcdData.state = APPGLCD_STATE_START_GLCD_UPDATE;
            }
            break;
        }
        case APPGLCD_STATE_START_GLCD_UPDATE:
        {
            if (IsSPI1TaskIdle ()) //I wait until the SERCOM1 task is idle
            {
                if (appglcdData.pointerY < (48/8))
                {
                    LCDSend(0x80, SEND_CMD);
                    appglcdData.state = APPGLCD_STATE_WRITE_Y_COORDINATES;  
                }
                else
                {
                    appglcdData.state = APPGLCD_STATE_IDLE; 
                }
            }
            break;
        }
        case APPGLCD_STATE_WRITE_Y_COORDINATES:
        {
            if (IsSPI1TaskIdle ()) //I wait until the SERCOM1 task is idle
            {
                LCDSend(0x40 | (uint8_t)(appglcdData.pointerY), SEND_CMD);
                appglcdData.state = APPGLCD_STATE_WRITE_X_COORDINATES; 
                appglcdData.pointerX = 0x00;
            }
            break;
        }
        case APPGLCD_STATE_WRITE_X_COORDINATES:
        {
            if (IsSPI1TaskIdle ()) //I wait until the SERCOM1 task is idle
            {
                if (appglcdData.pointerX < 84)
                {
                    LCDSend(appglcdData.LcdMemory[appglcdData.pointerY * 84 + appglcdData.pointerX], SEND_CHR);
                    appglcdData.pointerX++;
                }
                else
                {
                    appglcdData.state = APPGLCD_STATE_START_GLCD_UPDATE;
                    appglcdData.pointerY++;
                }
            }
            break;
        }
        case APPGLCD_STATE_IDLE: break;
        /* The default state should never be executed. */
        default: break; /* TODO: Handle error in application's state machine. */
    }
}


/*******************************************************************************
 End of File
 */
