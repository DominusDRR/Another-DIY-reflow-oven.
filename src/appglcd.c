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

bool IsGLCDTaskIdle (void);
void LCDPixelXY(uint32_t x, uint32_t y);
void LCDLine (int32_t x1, int32_t y1, int32_t x2, int32_t y2);
/* TODO:  Add any necessary local functions.
*/
/****************************************************************************/
/*  Determines whether the GLCD module task is idle so it can send or receive 
 *  data */                                                             
/*  Function : IsGLCDTaskIdle                                               */
/*  Parameters                                                              */
/*  Input   :  Nothing                                                      */
/*  Output  :  true or false                                                */
/****************************************************************************/
bool IsGLCDTaskIdle (void)
{
    if (APPGLCD_STATE_IDLE == appglcdData.state)
    {
        return true;
    }
    return false;
}
/****************************************************************************/
/*  Send to LCD                                                             */
/*  Function : LCDPixelXY                                                   */
/*  Parameters                                                              */
/*  Input   :  x and y coordinates where you want to graph a point          */
/*  Output  :  Nothing                                                      */
/****************************************************************************/
void LCDPixelXY(uint32_t x, uint32_t y)
{
    uint32_t index = 0;
    //uint16_t i = 0;
    
    // check for out off range
    if ((x > LCD_X_RES)||(x < 0)) return;
    if ((y > LCD_Y_RES)||(y < 0)) return;

    index = x + ((y/8))*84 ;
    appglcdData.LcdMemory[index] |= (uint8_t)(1<<(y%8));
    //The process of updating GLCD information begins
    appglcdData.pointerY1 = 0x00;
    appglcdData.state = APPGLCD_STATE_START_GLCD_UPDATE;
}
/****************************************************************************/
/*  Send to LCD                                                             */
/*  Function : LCDLine                                                      */
/*  Parameters                                                              */
/*  Input   :  x1, x2 and y1, y2 coordinates where you want to graph a line */
/*  Output  :  Nothing                                                      */
/****************************************************************************/
void LCDLine (int32_t x1, int32_t y1, int32_t x2, int32_t y2) //draw a line
{      
    appglcdData.dx = abs (x2 - x1);
    appglcdData.dy = abs (y2 - y1);
    if ( x1 < x2) 
    {
        appglcdData.sx = 1;
    }
    else 
    {    
        appglcdData.sx = -1;
    }
    if (y1 < y2) 
    {
        appglcdData.sy = 1;
    }
    else 
    {    
        appglcdData.sy = -1;
    }
    
    appglcdData.error1 = appglcdData.dx - appglcdData.dy;
    appglcdData.pointerX1 = x1;
    appglcdData.pointerY1 = y1;
    appglcdData.pointerX2 = x2;
    appglcdData.pointerY2 = y2;
    appglcdData.state = APPGLCD_STATE_GRAPH_LINE;
}

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
                appglcdData.pointerX1 = 0x00;
                appglcdData.state = APPGLCD_STATE_LCD_COMMANDS;
            }
            break;
        }
        /* TODO: implement your application state machine.*/
        case APPGLCD_STATE_LCD_COMMANDS:
        {
            if (IsSPI1TaskIdle ()) //I wait until the SERCOM1 task is idle
            {
                LCDSend(commandsInitializeGLCD[appglcdData.pointerX1], SEND_CMD);
                appglcdData.pointerX1++;
                if (appglcdData.pointerX1 >= sizeof(commandsInitializeGLCD))
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
                appglcdData.pointerX1 = 0x00;
                appglcdData.state = APPGLCD_STATE_CLEAR_LCD;
            }
            break;
        }
        case APPGLCD_STATE_CLEAR_LCD:
        {
            if (appglcdData.pointerX1 < sizeof(appglcdData.LcdMemory))
            {
                appglcdData.LcdMemory[appglcdData.pointerX1] = 0x00;
                appglcdData.pointerX1++;
            }
            else
            {
                appglcdData.pointerY1 = 0x00;
                appglcdData.state = APPGLCD_STATE_START_GLCD_UPDATE;
            }
            break;
        }
        case APPGLCD_STATE_START_GLCD_UPDATE:
        {
            if (IsSPI1TaskIdle ()) //I wait until the SERCOM1 task is idle
            {
                if (appglcdData.pointerY1 < (48/8))
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
                LCDSend(0x40 | (uint8_t)(appglcdData.pointerY1), SEND_CMD);
                appglcdData.state = APPGLCD_STATE_WRITE_X_COORDINATES; 
                appglcdData.pointerX1 = 0x00;
            }
            break;
        }
        case APPGLCD_STATE_WRITE_X_COORDINATES:
        {
            if (IsSPI1TaskIdle()) //I wait until the SERCOM1 task is idle
            {
                if (appglcdData.pointerX1 < 84)
                {
                    LCDSend(appglcdData.LcdMemory[appglcdData.pointerY1 * 84 + appglcdData.pointerX1], SEND_CHR);
                    appglcdData.pointerX1++;
                }
                else
                {
                    appglcdData.state = APPGLCD_STATE_START_GLCD_UPDATE;
                    appglcdData.pointerY1++;
                }
            }
            break;
        }
        case APPGLCD_STATE_IDLE: break;
        case APPGLCD_STATE_GRAPH_LINE:
        {
            if (IsSPI1TaskIdle()) //I wait until the SERCOM1 task is idle
            {
                LCDPixelXY (appglcdData.pointerX1, appglcdData.pointerY1);
                if ((appglcdData.pointerX1 == appglcdData.pointerX2) && (appglcdData.pointerY1 == appglcdData.pointerY2))
                {
                    appglcdData.state = APPGLCD_STATE_WAIT_PROCESS_COMPLETION_GLCD; 
                }
                else
                {
                    appglcdData.error2 = 2*appglcdData.error1;
                    if (appglcdData.error2 > -appglcdData.dy) 
                    {
                        appglcdData.error1 = appglcdData.error1 - appglcdData.dy;
                        appglcdData.pointerX1 = appglcdData.pointerX1 + appglcdData.sx;
                    }
                    if (appglcdData.error2 < appglcdData.dx) 
                    {
                        appglcdData.error1 = appglcdData.error1 + appglcdData.dx;
                        appglcdData.pointerY1 = appglcdData.pointerY1 + appglcdData.sy;
                    }
                }
            }
            break;
        }
        case APPGLCD_STATE_WAIT_PROCESS_COMPLETION_GLCD:
        {
            if (IsSPI1TaskIdle()) //I wait until the SERCOM1 task is idle
            {
                appglcdData.state = APPGLCD_STATE_IDLE; 
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
