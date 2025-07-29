/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    appmax.c

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

#include "appmax.h"
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
    This structure should be initialized by the APPMAX_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APPMAX_DATA appmaxData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/
extern uint32_t abs_diff_uint32(uint32_t a, uint32_t b);

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************
static void SPI0Instance1EventHandler (DRV_SPI_TRANSFER_EVENT event, DRV_SPI_TRANSFER_HANDLE transferHandle, uintptr_t context);
uint8_t decodeThermocoupleError(uint8_t rawByte);
/* TODO:  Add any necessary local functions.
*/
static void SPI0Instance1EventHandler (DRV_SPI_TRANSFER_EVENT event, DRV_SPI_TRANSFER_HANDLE transferHandle, uintptr_t context)
{
    if (event == DRV_SPI_TRANSFER_EVENT_COMPLETE)
    {
        appmaxData.isTransferComplete = true;
    }
    else
    {
        appmaxData.isTransferComplete = false;
        appmaxData.typeOfError = SPI0_TRANSMISSION;
        appmaxData.state = APPMAX_STATE_ERROR;
    }
}
uint8_t decodeThermocoupleError(uint8_t rawByte)
{
    uint8_t err = rawByte & 0x07;
    if (err & 0x01) return OPEN_THERMOCOUPLE;
    if (err & 0x02) return SHORT_CIRCUIT_TO_GND;
    if (err & 0x04)  return SHORT_CIRCUIT_TO_VCC;
    return NO_ERROR_THERMOCOUPLE;
}
uint8_t getTypeThermocoupleError(void)
{
    return appmaxData.typeThermocoupleError;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APPMAX_Initialize ( void )

  Remarks:
    See prototype in appmax.h.
 */

void APPMAX_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appmaxData.state = APPMAX_STATE_INIT;
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    SS0_Set();
    appmaxData.drvSPIHandle = DRV_HANDLE_INVALID;
    appmaxData.typeOfError = SPI1_NO_ERROR;
}


/******************************************************************************
  Function:
    void APPMAX_Tasks ( void )

  Remarks:
    See prototype in appmax.h.
 */

void APPMAX_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appmaxData.state )
    {
        /* Application's initial state. */
        case APPMAX_STATE_INIT:
        {
            appmaxData.setup.baudRateInHz    = 1000000;
            appmaxData.setup.clockPhase      = DRV_SPI_CLOCK_PHASE_VALID_LEADING_EDGE;
            appmaxData.setup.clockPolarity   = DRV_SPI_CLOCK_POLARITY_IDLE_LOW;
            appmaxData.setup.dataBits        = DRV_SPI_DATA_BITS_8;
            appmaxData.setup.chipSelect      = SS0_PIN;
            appmaxData.setup.csPolarity      = DRV_SPI_CS_POLARITY_ACTIVE_LOW;
            appmaxData.state  = APPMAX_STATE_DRIVER_SETUP;
            break;
        }
        case APPMAX_STATE_DRIVER_SETUP:
        {
            appmaxData.drvSPIHandle = DRV_SPI_Open( DRV_SPI_INDEX_1, DRV_IO_INTENT_READWRITE );
            if(appmaxData.drvSPIHandle != DRV_HANDLE_INVALID)
            {
                if(DRV_SPI_TransferSetup(appmaxData.drvSPIHandle, &appmaxData.setup) == true)
                {
                    DRV_SPI_TransferEventHandlerSet(appmaxData.drvSPIHandle, SPI0Instance1EventHandler, (uintptr_t)0);
                    appmaxData.typeOfError = SPI0_NO_ERROR; //If it comes from APPSPI1_STATE_DELAY_TO_REOPEN_SPI state, it needs to be cleared
                    appmaxData.adelay = RTC_Timer32CounterGet();
                    appmaxData.state = APPMAX_STATE_MEASURE_TEMPERATURE_EVERY_500ms;
                }
                else
                {
                    appmaxData.typeOfError = SPI0_SET_INSTANCE;
                    appmaxData.state = APPMAX_STATE_ERROR;
                }
            }
            else
            {
                appmaxData.typeOfError = SPI0_OPEN_ERROR;
                appmaxData.state = APPMAX_STATE_ERROR;
            }    
            break;
        }
        /* TODO: implement your application state machine.*/
        case APPMAX_STATE_MEASURE_TEMPERATURE_EVERY_500ms:
        {
            if (abs_diff_uint32(RTC_Timer32CounterGet(), appmaxData.adelay) > _500ms)
            {
                appmaxData.state = APPMAX_WAIT_FOR_RECEPTION_END;
                SS0_Clear();
                DRV_SPI_ReadTransferAdd(appmaxData.drvSPIHandle,appmaxData.bufferRX,0x04,&appmaxData.transferHandle);
                if(appmaxData.transferHandle == DRV_SPI_TRANSFER_HANDLE_INVALID)
                {
                    appmaxData.state = APPMAX_STATE_ERROR;
                }
            }
            break;
        }
        case APPMAX_WAIT_FOR_RECEPTION_END:
        {
            if (appmaxData.isTransferComplete == true)
            {
                SS0_Set();
                appmaxData.isTransferComplete = false;
                if (SPI0_NO_ERROR == appmaxData.typeOfError)
                {   
                    appmaxData.adelay = RTC_Timer32CounterGet();
                    appmaxData.typeThermocoupleError = decodeThermocoupleError(appmaxData.bufferRX[0x03]);
                    appmaxData.state = APPMAX_STATE_MEASURE_TEMPERATURE_EVERY_500ms;
                }
            }
            break;
        }
        case APPMAX_STATE_ERROR:
        {
            SS0_Set();
            switch(appmaxData.typeOfError)
            {
                case SPI0_OPEN_ERROR:
                case SPI0_SET_INSTANCE:    
                {
                    appmaxData.adelay = RTC_Timer32CounterGet();
                    appmaxData.state = APPMAX_STATE_DELAY_TO_REOPEN_SPI;
                    break;
                }
                default:
                {
                    DRV_SPI_Close(appmaxData.drvSPIHandle);
                    APPMAX_Initialize();
                }    
            }        
            break;
        }
        case APPMAX_STATE_DELAY_TO_REOPEN_SPI:
        {
            if ( abs_diff_uint32(RTC_Timer32CounterGet(), appmaxData.adelay) > _500ms)
            {
                appmaxData.state = APPMAX_STATE_DRIVER_SETUP;
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
