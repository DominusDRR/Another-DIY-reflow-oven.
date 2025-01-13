/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    appeeram.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APPEERAM_Initialize" and "APPEERAM_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APPEERAM_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

#ifndef _APPEERAM_H
#define _APPEERAM_H

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
#include "driver/i2c/drv_i2c.h"
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
#define I2C2_NO_ERROR                                               0x00
#define I2C2_OPEN_ERROR                                             0x01
#define I2C2_ASSIGN_READ_HANDLER_ASE_ERROR                          0x02
#define I2C2_COMPLETE_READ_TRANSFER_ASE_ERROR                       0x03
#define I2C2_ASSIGN_WRITE_HANDLER_ASE_ERROR                         0x04
#define I2C2_COMPLETE_WRITE_TRANSFER_ASE_ERROR                      0x05
#define I2C2_ASSIGN_WRITE_HANDLER_INITIAL_ADDRESS_ERROR             0x06
#define I2C2_COMPLETE_WRITE_TRANSFER_INITIAL_ADDRESS_ERROR          0x07
#define I2C2_ASSIGN_READ_HANDLER_EERAM_ARRAY_ERROR                  0x08  
#define I2C2_EERAM_ILLOLOGICAL_DATA_ERROR                           0x09    

#define ERROR_COMPLETE_READ_TRANSFER_EERAM_ARRAY                    0x09
#define ERROR_ASSIGN_WRITE_HANDLER_EERAM_ARRAY                      0x0A
#define ERROR_COMPLETE_WRITE_TRANSFER_EERAM_ARRAY                   (0x0B + 0x01)
    
    
    
/**  EERAM memory command definitions  **/
#define EERAMStatusByte             0x30
#define StatusByteAddress           0x00
#define StatusByteValue             0x02  // ASE only activated, no protection
#define ByteControlWriteEERAM       0xA0
#define ByteControlReadEERAM        0xA1

#define MAXIMUM_BUFFER_RECEIVE      40
#define MAXIMUM_BUFFER_TRANSMISSION 40     
#define APPEERAM_NUMBER_BYTE_READ   24
    
    
typedef enum
{
    APPEERAM_TRANSFER_STATUS_IN_PROGRESS,
    APPEERAM_TRANSFER_STATUS_SUCCESS,
    APPEERAM_TRANSFER_STATUS_ERROR,
    APPEERAM_TRANSFER_STATUS_IDLE,
} APPEERAM_TRANSFER_STATUS;    
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
    APPEERAM_STATE_INIT=0,
    APPEERAM_STATE_HAS_ELAPSED_25ms,
    /* TODO: Define states used by the application state machine. */
    APPEERAM_STATE_READ_BIT_ASE, 
    APPEERAM_STATE_ANALYZE_BIT_ASE,
    APPEERAM_STATUS_ACTIVATE_BIT_ASE,
    APPEERAM_STATUS_WAIT_WRITE_BIT_ASE,        
    APPEERAM_STATUS_WRITE_INITIAL_ADDRESS_READ,    
    APPEERAM_STATUS_READ_ARRAY_EERAM,   
    APPEERAM_STATE_INITIALIAR_CRC,    
    APPEERAM_STATUS_WAIT_CRC, 
    APPEERAM_STATUS_ANALYZE_DATA_READ_FROM_EERAM,
    APPEERAM_STATE_DEFAULT_VALUES,        
    APPEERAM_STATE_IDLE,        
    APPEERAM_STATE_ERROR,
} APPEERAM_STATES;


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
    APPEERAM_STATES state;
    /* TODO: Define any additional data used by the application. */
    uint32_t adelay;
    DRV_HANDLE drvI2CHandle;
    DRV_I2C_TRANSFER_HANDLE transferHandle;
    volatile APPEERAM_TRANSFER_STATUS transferStatus;
    uint8_t typeOfError;
    uint8_t errorAPPEERAM;
    APPEERAM_STATES stateWhereToJumpAfterCRC;
    uint8_t attempts;
    uint8_t  temperatureA;
    uint16_t timeA;
    uint8_t  temperatureB;
    uint16_t timeB;
    uint8_t  temperatureC;
    uint16_t timeC;
    uint8_t  temperatureD;
    uint16_t timeD;
    float   Kp;
    float   Ki;
    float   Kd;
} APPEERAM_DATA;

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
    void APPEERAM_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the
    application in its initial state and prepares it to run so that its
    APPEERAM_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APPEERAM_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APPEERAM_Initialize ( void );


/*******************************************************************************
  Function:
    void APPEERAM_Tasks ( void )

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
    APPEERAM_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APPEERAM_Tasks( void );

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* _APPEERAM_H */

/*******************************************************************************
 End of File
 */

