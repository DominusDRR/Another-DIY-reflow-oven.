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
#include <stdio.h>
#include <string.h>
//#include <stdint.h>
//#include <stdbool.h>
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
#define TOTAL_ITEMS          (sizeof(msgsParametersMenu)/sizeof(*msgsParametersMenu)) // 8 messages
#define VISIBLE_ROWS         6
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

const unsigned char *msgsParametersMenu[] = 
{
    (unsigned char *)"1 Temp preheat", // max 14 chars
    (unsigned char *)"2 Time preheat", 
    (unsigned char *)"3 Temp flux act",
    (unsigned char *)"4 Time flux act",
    (unsigned char *)"5 Temp reflow",
    (unsigned char *)"6 Time reflow",
    (unsigned char *)"7 Temp cooling",
    (unsigned char *)"8 Time cooling",
    (unsigned char *)"9 Kp constant ",
    (unsigned char *)"10 Ki constant",
    (unsigned char *)"11 Kd constant"
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
extern void LCDChrXY_Scaled(uint8_t x, uint8_t y, const uint8_t *dataPtr, uint8_t scale, bool updateLCD);
extern void LCDStr_Scaled_Clear(uint8_t x, uint8_t y, uint8_t len, uint8_t scale, bool updateLCD);
extern void LCDTinyStr(uint8_t x, uint8_t y, const char *dataPtr, bool updateLCD);
extern void drawInitialLogo(void);

extern uint32_t abs_diff_uint32(uint32_t a, uint32_t b);
extern uint8_t getPressedBtn(void);

extern uint8_t getTypeThermocoupleError(void);
extern float getThermocoupleTemp(void);
extern float getInternalTemp(void);
extern bool IsMaxTaskIdle (void);
extern void startTemperatureReading(void);

extern uint16_t returnParameterTempTime(uint8_t index);
extern float returnConstantsPID(uint8_t index);
extern uint16_t increaseParameterTempTime(uint8_t index);
extern uint16_t decreaseParameterTempTime(uint8_t index); 
// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************
void returnHomeMenu(void);
void goParametersMenu(void);
char isTemperatureOrTime(uint8_t index);
/* TODO:  Add any necessary local functions.
*/
void returnHomeMenu(void)
{
    apphmiData.typeErrorThermocuple = 0xFF; 
    apphmiData.selectedOption = 0x02;
    apphmiData.doNotClearLCD = false; //This flag can be set to true, so it is cleared for the proxy.
    apphmiData.state = APPHMI_STATE_CLEAR_LCD;
}
void goParametersMenu(void)
{
    LCDClear();
    apphmiData.messagePointer = 0x00;
    apphmiData.selectedOption = 0x00;
    apphmiData.firstVisibleIndex = 0x00;
    apphmiData.state =  APPHMI_STATE_REQUEST_START_SET_PARAMETERS;
}
char isTemperatureOrTime(uint8_t index)
{
    return ((index & 1) == 0) ? 'C' : 's';
}
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
    apphmiData.typeErrorThermocuple = 0xFF; //This way I force the thermocouple status to update for the first time.
    apphmiData.doNotClearLCD = false;
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
                    apphmiData.stateToReturn = APPHMI_STATE_WAIT_USER_SELECTION_HOME_MENU;
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
                apphmiData.state = apphmiData.stateToReturn;
            }
            break;
        }
        case APPHMI_STATE_WAIT_USER_SELECTION_HOME_MENU:
        {
            uint8_t button = getPressedBtn();
            if (NO_BUTTON_PRESSED != button && IsGLCDTaskIdle())
            {
                switch (button)
                {
                    case OK_BUTTON_PRESSED:
                    {
                        if (0x04 == apphmiData.selectedOption)
                        {
                            apphmiData.adelay = RTC_Timer32CounterGet();
                            apphmiData.state = APPHMI_STATE_REQUEST_START_TEMPERATURE_READING;
                        }
                        else if (0x03 == apphmiData.selectedOption) // To parameters Menu
                        {
                            goParametersMenu();
                        }
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
        /*** Current temperature display ***/
        case APPHMI_STATE_REQUEST_START_TEMPERATURE_READING:
        {
            if ( abs_diff_uint32(RTC_Timer32CounterGet(), apphmiData.adelay) > _500ms)
            {
                if (IsMaxTaskIdle())
                {
                    startTemperatureReading();
                    apphmiData.state = APPHMI_STATE_WAIT_TEMPERATURE_READING;
                }
                else
                {
                    apphmiData.adelay = RTC_Timer32CounterGet();
                }
            }
            if (ESC_BUTTON_PRESSED == getPressedBtn()) // Return to the HOME menu
            {
                returnHomeMenu();
            }
            break;
        }
        case APPHMI_STATE_WAIT_TEMPERATURE_READING:
        {
            if (IsMaxTaskIdle())
            {
                apphmiData.state = APPHMI_STATE_START_WRITE_CURRENT_TEMPERATURE;
            }
            break;
        }
        case APPHMI_STATE_START_WRITE_CURRENT_TEMPERATURE:
        {
            if (IsGLCDTaskIdle()) // I wait until the GLCD task is idle
            {
                if (apphmiData.typeErrorThermocuple != getTypeThermocoupleError())
                {
                    apphmiData.typeErrorThermocuple = getTypeThermocoupleError();
                    if (apphmiData.doNotClearLCD)
                    {
                        apphmiData.doNotClearLCD = false;
                    }
                    else
                    {
                        LCDClear();
                    }
                    apphmiData.state = APPHMI_STATE_DISPLAY_THERMOCOUPLE_ERRORS;
                    return; //So that the if below is not executed
                }
                apphmiData.adelay = RTC_Timer32CounterGet();
                apphmiData.state = APPHMI_STATE_REQUEST_START_TEMPERATURE_READING;
            }
            break;
        }
        case APPHMI_STATE_DISPLAY_THERMOCOUPLE_ERRORS:
        {
            if (IsGLCDTaskIdle()) // I wait until the GLCD task is idle
            {
                switch (apphmiData.typeErrorThermocuple)
                {
                    case OPEN_THERMOCOUPLE:     LCDStr(0x02,(unsigned char *)"Thermocouple  open",false, true);     break;
                    case SHORT_CIRCUIT_TO_GND:  LCDStr(0x02,(unsigned char *)"Thermocouple  to GNC",false, true);   break;
                    case SHORT_CIRCUIT_TO_VCC:  LCDStr(0x02,(unsigned char *)"Thermocouple  to Vcc",false, true);   break;
                    default:                    apphmiData.state = APPHMI_STATE_GET_THERMOCUPLE_TEMPERATUE;         return;
                }
                apphmiData.adelay = RTC_Timer32CounterGet();
                apphmiData.state = APPHMI_STATE_REQUEST_START_TEMPERATURE_READING;
            }
            break;
        }
        case APPHMI_STATE_GET_THERMOCUPLE_TEMPERATUE://Split into several states, so that the task does not completely take over the CPU
        {
            apphmiData.thermocoupleTemp = getThermocoupleTemp();
            apphmiData.state = APPHMI_STATE_GET_INTERNAL_TEMPERATUE;
            break;
        }
        case APPHMI_STATE_GET_INTERNAL_TEMPERATUE:
        {
            apphmiData.internalTemp = getInternalTemp();
            apphmiData.state = APPHMI_STATE_WRITE_CURRENT_TEMPERATURE;
            break;
        }
        case APPHMI_STATE_WRITE_CURRENT_TEMPERATURE:
        {
            if (IsGLCDTaskIdle()) // I wait until the GLCD task is idle
            {
                snprintf(apphmiData.bufferForStrings, sizeof(apphmiData.bufferForStrings),"Thermocuple   Temp %.1f C", apphmiData.thermocoupleTemp);
                LCDStr(0, (uint8_t*)apphmiData.bufferForStrings, false, false);
                apphmiData.state = APPHMI_STATE_WRITE_INTERNAL_TEMPERATURE;
            }
            break;
        }
        case APPHMI_STATE_WRITE_INTERNAL_TEMPERATURE:
        {
            if (IsGLCDTaskIdle()) // I wait until the GLCD task is idle
            {
                //static char buf2[30];
                snprintf(apphmiData.bufferForStrings, sizeof(apphmiData.bufferForStrings),"Internal      Temp %.1f C", apphmiData.internalTemp);
                LCDStr(3, (uint8_t*)apphmiData.bufferForStrings, false, false);
                apphmiData.state = APPHMI_STATE_UPDATE_TEMPERATURES;
            }
            break;
        }
        case APPHMI_STATE_UPDATE_TEMPERATURES:
        {
            if (IsGLCDTaskIdle()) // I wait until the GLCD task is idle
            {
                LCDUpdate();
                apphmiData.adelay = RTC_Timer32CounterGet();
                apphmiData.typeErrorThermocuple = 0xFF; //I do this so that another temperature
                apphmiData.doNotClearLCD = true; //I prevent the screen from being cleaned, it produces a flickering and does not look natural.
                apphmiData.state = APPHMI_STATE_REQUEST_START_TEMPERATURE_READING; 
            }
            break;
        }
        /*****************************************************************************************************************/
        /** MENU SET PARAMETERS **/
        case APPHMI_STATE_REQUEST_START_SET_PARAMETERS:
        {
            if (IsGLCDTaskIdle()) // I wait until the GLCD task is idle
            {
                if (apphmiData.messagePointer < VISIBLE_ROWS)
                {
                    uint8_t idx = apphmiData.firstVisibleIndex + apphmiData.messagePointer;
                    if (idx < TOTAL_ITEMS) //VISIBLE_ROWS = 6
                    {
                        LCDStr(apphmiData.messagePointer, msgsParametersMenu[idx],(idx == apphmiData.selectedOption), false);
                    }
                    apphmiData.messagePointer++;
                }
                else
                {
                    apphmiData.messagePointer = 0x00;
                    apphmiData.stateToReturn = APPHMI_STATE_WAI_USER_SELECTION_PARAMETERS_MENU;
                    apphmiData.state = APPHMI_STATE_UPDATE_HOME_MENU;
                }
            }
            break;
        }
        case APPHMI_STATE_WAI_USER_SELECTION_PARAMETERS_MENU:
        {
            uint8_t button = getPressedBtn();
            if (NO_BUTTON_PRESSED != button)
            {
                switch (button)
                {
                    case OK_BUTTON_PRESSED:
                    {
                        apphmiData.state = APPHMI_STATE_START_EDIT_PARAMETER;
                        break;
                    }
                    case UP_BUTTON_PRESSED:
                    {   
                        if (apphmiData.selectedOption > 0)
                        {
                            apphmiData.selectedOption--;
                            if (apphmiData.selectedOption < apphmiData.firstVisibleIndex)
                            {
                                apphmiData.firstVisibleIndex = apphmiData.selectedOption;
                            }
                            apphmiData.state = APPHMI_STATE_REQUEST_START_SET_PARAMETERS;
                        }       
                        break;
                    }
                    case DOWN_BUTTON_PRESSED:
                    {
                        if (apphmiData.selectedOption < (TOTAL_ITEMS - 1))
                        {
                            apphmiData.selectedOption++;
                            if (apphmiData.selectedOption >= apphmiData.firstVisibleIndex + VISIBLE_ROWS)// 0 +6
                            {
                                apphmiData.firstVisibleIndex = apphmiData.selectedOption - (VISIBLE_ROWS - 1); //= 6 - 6 + 1 = 1
                            }
                            apphmiData.state = APPHMI_STATE_REQUEST_START_SET_PARAMETERS;
                        }
                        break;
                    }
                    default: returnHomeMenu();
                }
            }
            break;
        }
        case APPHMI_STATE_START_EDIT_PARAMETER:
        {
            if (IsGLCDTaskIdle()) // I wait until the GLCD task is idle
            {
                LCDClear();
                apphmiData.state = APPHMI_STATE_SET_NAME_PARAMETER;
            }
            break;
        }
        case APPHMI_STATE_SET_NAME_PARAMETER:
        {
            if (IsGLCDTaskIdle()) // I wait until the GLCD task is idle
            {
                //LCDTinyStr(0, 27,"Max: 32.8C", false);
                strncpy(apphmiData.bufferForStrings, (const char *)msgsParametersMenu[apphmiData.selectedOption], 14);
                apphmiData.bufferForStrings[14] = '\0';  // aseguramos que esté terminado en null
                LCDStr(0,(uint8_t *)apphmiData.bufferForStrings,false, false);
                apphmiData.state = APPHMI_STATE_SET_VALUE_PARAMETER;
            }
            break;
        }
        case APPHMI_STATE_SET_VALUE_PARAMETER:
        {
            if (IsGLCDTaskIdle()) // I wait until the GLCD task is idle
            {
                if (apphmiData.selectedOption < 0x08)
                {
                    sprintf(apphmiData.bufferForStrings,"%u %c",returnParameterTempTime(apphmiData.selectedOption),isTemperatureOrTime(apphmiData.selectedOption));
                }
                else
                {
                    sprintf(apphmiData.bufferForStrings, "%.1f", returnConstantsPID(apphmiData.selectedOption));
                }
                LCDChrXY_Scaled(5,15,(uint8_t *)apphmiData.bufferForStrings,2,false);
                apphmiData.state = APPHMI_STATE_UPDATE_LCD_VALUE_PARAMETER;
            }
            break;
        }
        case APPHMI_STATE_UPDATE_LCD_VALUE_PARAMETER:
        {
            if (IsGLCDTaskIdle()) // I wait until the GLCD task is idle
            {
                LCDUpdate();
                apphmiData.state = APPHMI_STATE_WAIT_USER_CHANGE_PARAMETERS;
            }
            break;
        }
        case APPHMI_STATE_WAIT_USER_CHANGE_PARAMETERS:
        {
            uint8_t button = getPressedBtn();
            if (NO_BUTTON_PRESSED != button && IsGLCDTaskIdle())
            {
                switch (button)
                {
                    case ESC_BUTTON_PRESSED: goParametersMenu(); break;
                    case UP_BUTTON_PRESSED:
                    case DOWN_BUTTON_PRESSED:    
                    {
                        LCDStr_Scaled_Clear(5,15,5,2,true);
                        apphmiData.doNotClearLCD = false;//With that boolean, I will discern whether it is an increase or decrease.
                        if (UP_BUTTON_PRESSED == button)
                        {
                            apphmiData.doNotClearLCD = true;
                        }
                        apphmiData.state = APPHMI_STATE_WAIT_ESCALATED_MESSAGE_CLEAN;
                        break;
                    }
                    default: break;    
                }
                
            }
            break;
        }
        case APPHMI_STATE_WAIT_ESCALATED_MESSAGE_CLEAN://The LCDChrXY_Scaled function does not clear the above, since it acts directly on each pixel on the screen, not at the byte level in the LCDBuffer.
        {
            if (IsGLCDTaskIdle()) // I wait until the GLCD task is idle
            {
                if (apphmiData.selectedOption < 0x08)
                {
                    //sprintf(apphmiData.bufferForStrings, "%u C", increaseParameterTempTime(apphmiData.selectedOption));  // o snprintf(buffer, sizeof(buffer), "%u", valor);
                    if (apphmiData.doNotClearLCD)
                    {
                        apphmiData.doNotClearLCD = false; //It is important that it remains false after using it.
                        sprintf(apphmiData.bufferForStrings,"%u %c",increaseParameterTempTime(apphmiData.selectedOption),isTemperatureOrTime(apphmiData.selectedOption));
                    }
                    else
                    {
                        sprintf(apphmiData.bufferForStrings,"%u %c",decreaseParameterTempTime(apphmiData.selectedOption),isTemperatureOrTime(apphmiData.selectedOption));
                    }
                }
                LCDChrXY_Scaled(5,15,(uint8_t *)apphmiData.bufferForStrings,2,true);//set to true to update the LCD immediately
                apphmiData.state = APPHMI_STATE_WAIT_USER_CHANGE_PARAMETERS;
            }
        }
        /**************************************************************/
        /* The default state should never be executed. */
        default: break; /* TODO: Handle error in application's state machine. */
    }
}


/*******************************************************************************
 End of File
 */
