/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    appspi1.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APPSPI1_Initialize" and "APPSPI1_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APPSPI1_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

#ifndef _APPSPI1_H
#define _APPSPI1_H

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
#include "driver/spi/drv_spi.h"

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
#define  SPI1_NO_ERROR       0x00
#define  SPI1_OPEN_ERROR     0x01
#define  SPI1_TRANSMISSION   0x02       
#define  SPI1_HANLDER_ERROR  0x03  
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
    APPSPI1_STATE_INIT=0,
    APPSPI1_STATE_DRIVER_SETUP,
    /* TODO: Define states used by the application state machine. */
    APPSPI1_STATE_IDLE,
    APPSPI1_START_TRANSMISSION,
    APPSPI1_TRANSMISSION,
    APPSPI1_WAIT_FOR_TRANSMISSION_END,        
    APPSPI1_STATE_ERROR,
    APPSPI1_STATE_DELAY_TO_REOPEN_SPI
} APPSPI1_STATES;


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
    APPSPI1_STATES state;
    /* TODO: Define any additional data used by the application. */
    DRV_HANDLE drvSPIHandle;
    DRV_SPI_TRANSFER_HANDLE transferHandle;
    DRV_SPI_TRANSFER_SETUP  setup; 
    uint8_t typeOfError;
    uint8_t bufferTX[32]; //Buffer for transmission
    uint8_t numberBytesTransm; //number of bytes to be transmitted
    bool isTransferComplete;
    uint32_t adelay;
} APPSPI1_DATA;

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
    void APPSPI1_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the
    application in its initial state and prepares it to run so that its
    APPSPI1_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APPSPI1_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APPSPI1_Initialize ( void );


/*******************************************************************************
  Function:
    void APPSPI1_Tasks ( void )

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
    APPSPI1_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APPSPI1_Tasks( void );

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* _APPSPI1_H */

/*******************************************************************************
 End of File
 */

