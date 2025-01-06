/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    appspi1.c

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

#include "appspi1.h"

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
    This structure should be initialized by the APPSPI1_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APPSPI1_DATA appspi1Data;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

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
    void APPSPI1_Initialize ( void )

  Remarks:
    See prototype in appspi1.h.
 */

void APPSPI1_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appspi1Data.state = APPSPI1_STATE_INIT;
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    SS1_Set();
    appspi1Data.drvSPIHandle = DRV_HANDLE_INVALID;
}


/******************************************************************************
  Function:
    void APPSPI1_Tasks ( void )

  Remarks:
    See prototype in appspi1.h.
 */

void APPSPI1_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( appspi1Data.state )
    {
        /* Application's initial state. */
        case APPSPI1_STATE_INIT:
        {
            appspi1Data.setup.baudRateInHz    = 1000000;
            appspi1Data.setup.clockPhase      = DRV_SPI_CLOCK_PHASE_VALID_LEADING_EDGE;
            appspi1Data.setup.clockPolarity   = DRV_SPI_CLOCK_POLARITY_IDLE_LOW;
            appspi1Data.setup.dataBits        = DRV_SPI_DATA_BITS_8;
            appspi1Data.setup.chipSelect      = SS1_PIN;
            appspi1Data.setup.csPolarity      = DRV_SPI_CS_POLARITY_ACTIVE_LOW;
            appspi1Data.state = APPSPI1_STATE_DRIVER_SETUP;
            break;
        }
        case APPSPI1_STATE_DRIVER_SETUP:
        {
            appspi1Data.drvSPIHandle = DRV_SPI_Open( DRV_SPI_INDEX_0, DRV_IO_INTENT_READWRITE );
            if(appspi1Data.drvSPIHandle != DRV_HANDLE_INVALID)
            {
                appspi1Data.state = APPSPI1_STATE_IDLE;
            }
            else
            {
                appspi1Data.typeOfError = SPI_OPEN_ERROR;
                appspi1Data.state = APPSPI1_STATE_ERROR;
            }    
            break;
        }
        /* TODO: implement your application state machine.*/
        case APPSPI1_STATE_IDLE: break; // It waits until another process wants to write or read information on the SPI bus.
        case APPSPI1_STATE_ERROR:
        {

            break;
        } 
        /* The default state should never be executed. */
        default: break;  /* TODO: Handle error in application's state machine. */
    }
}


/*******************************************************************************
 End of File
 */
