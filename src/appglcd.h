/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    appglcd.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APPGLCD_Initialize" and "APPGLCD_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APPGLCD_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

#ifndef _APPGLCD_H
#define _APPGLCD_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "configuration.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
#define LCD_X_RES                  84
#define LCD_Y_RES                  48

#define PIXEL_OFF                  0
#define PIXEL_ON                   1
#define PIXEL_XOR                  2

#define FONT_1X                    1
#define FONT_2X                    2   
// this is the buffer size
#define LCD_CACHE_SIZE             ((LCD_X_RES * LCD_Y_RES) / 8)
// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
    /* Application's state machine's initial state. */
    APPGLCD_STATE_INIT=0,
    APPGLCD_STATE_WAIT_1000ms,
    /* TODO: Define states used by the application state machine. */
    APPGLCD_STATE_LCD_COMMANDS,
    APPGLCD_STATE_START_CLEANING_GLCD,
    APPGLCD_STATE_CLEAR_LCD,
    APPGLCD_STATE_START_GLCD_UPDATE,
    APPGLCD_STATE_WRITE_Y_COORDINATES,
    APPGLCD_STATE_WRITE_X_COORDINATES,
    APPGLCD_STATE_IDLE,
    APPGLCD_STATE_GRAPH_LINE,
    APPGLCD_STATE_WAIT_PROCESS_COMPLETION_GLCD        
} APPGLCD_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    APPGLCD_STATES state;
    /* TODO: Define any additional data used by the application. */
    uint32_t adelay;
    size_t pointerX1;
    size_t pointerY1;
    size_t pointerX2;
    size_t pointerY2;
    size_t error1;
    size_t error2;
    int32_t dx, dy, sx, sy;
    uint8_t LcdMemory[LCD_CACHE_SIZE];
} APPGLCD_DATA;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APPGLCD_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the
    application in its initial state and prepares it to run so that its
    APPGLCD_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APPGLCD_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APPGLCD_Initialize ( void );


/*******************************************************************************
  Function:
    void APPGLCD_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APPGLCD_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APPGLCD_Tasks( void );

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* _APPGLCD_H */

/*******************************************************************************
 End of File
 */

