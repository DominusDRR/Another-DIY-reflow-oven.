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
    This structure should be initialized by the APPSPI1_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APPSPI1_DATA appspi1Data;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************
extern uint32_t abs_diff_uint32(uint32_t a, uint32_t b);
/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************
bool IsSPI1TaskIdle (void);
void LCDSend(unsigned char data, unsigned char cd);
static void SPI1Instance1EventHandler (DRV_SPI_TRANSFER_EVENT event, DRV_SPI_TRANSFER_HANDLE transferHandle, uintptr_t context);
/* TODO:  Add any necessary local functions.
*/
/****************************************************************************/
/*  Determines whether the SERCOM1 module task is idle so it can send or 
 * receive data                                                             */                                                             
/*  Function : IsSPI1TaskIdle                                               */
/*      Parameters                                                          */
/*          Input   :  Nothing                                              */
/*          Output  :  true or false                                        */
/****************************************************************************/
bool IsSPI1TaskIdle (void)
{
    if (APPSPI1_STATE_IDLE == appspi1Data.state)
    {        
        return true;
    }
    return false;
}
/****************************************************************************/
/*  Send to LCD                                                             */
/*  Function : LCDSend                                                      */
/*      Parameters                                                          */
/*          Input   :  data and  SEND_CHR or SEND_CMD                       */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDSend(unsigned char data, unsigned char cd)
{
//    SS1_Clear();
    if (cd == SEND_CHR)
    {
        LCD_D_Set();
    }
    else
    {
        LCD_D_Clear();
    }
    appspi1Data.bufferTX[0x00] = data;
    appspi1Data.numberBytesTransm = 0x01;
    // send data over SPI
    appspi1Data.state = APPSPI1_START_TRANSMISSION;
    //SEND_BYTE_SPI();
    //LCD_CS_HIGH();
}
static void SPI1Instance1EventHandler (DRV_SPI_TRANSFER_EVENT event, DRV_SPI_TRANSFER_HANDLE transferHandle, uintptr_t context)
{
    if (event == DRV_SPI_TRANSFER_EVENT_COMPLETE)
    {
        appspi1Data.isTransferComplete = true;
    }
    else
    {
        appspi1Data.isTransferComplete = false;
        appspi1Data.typeOfError = SPI1_TRANSMISSION;
        appspi1Data.state = APPSPI1_STATE_ERROR;
    }
}

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
    appspi1Data.typeOfError = SPI1_NO_ERROR;
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
                if(DRV_SPI_TransferSetup(appspi1Data.drvSPIHandle, &appspi1Data.setup) == true)
                {
                    DRV_SPI_TransferEventHandlerSet(appspi1Data.drvSPIHandle, SPI1Instance1EventHandler, (uintptr_t)0);
                    appspi1Data.typeOfError = SPI1_NO_ERROR; //If it comes from APPSPI1_STATE_DELAY_TO_REOPEN_SPI state, it needs to be cleared
                    appspi1Data.state = APPSPI1_STATE_IDLE;
                }
                else
                {
                    appspi1Data.typeOfError = SPI1_SET_INSTANCE;
                    appspi1Data.state = APPSPI1_STATE_ERROR;
                }
            }
            else
            {
                appspi1Data.typeOfError = SPI1_OPEN_ERROR;
                appspi1Data.state = APPSPI1_STATE_ERROR;
            }    
            break;
        }
        /* TODO: implement your application state machine.*/
        case APPSPI1_STATE_IDLE: break; // It waits until another process wants to write or read information on the SPI bus.
        case APPSPI1_START_TRANSMISSION:
        {
            SS1_Clear();
            DRV_SPI_TransferEventHandlerSet(appspi1Data.drvSPIHandle, SPI1Instance1EventHandler, (uintptr_t)0);
            appspi1Data.state = APPSPI1_TRANSMISSION;
            break;
        }
        case APPSPI1_TRANSMISSION:
        {
            appspi1Data.state = APPSPI1_WAIT_FOR_TRANSMISSION_END;
            DRV_SPI_WriteTransferAdd(appspi1Data.drvSPIHandle, appspi1Data.bufferTX, appspi1Data.numberBytesTransm, &appspi1Data.transferHandle );
            if(appspi1Data.transferHandle == DRV_SPI_TRANSFER_HANDLE_INVALID)
            {
                appspi1Data.typeOfError = SPI1_HANLDER_ERROR;
                appspi1Data.state = APPSPI1_STATE_ERROR;
            }
            break;
        }
        case APPSPI1_WAIT_FOR_TRANSMISSION_END:
        {
            if (appspi1Data.isTransferComplete == true)
            {
                SS1_Set();
                appspi1Data.isTransferComplete = false;
                if (SPI1_NO_ERROR == appspi1Data.typeOfError)
                {    
                    appspi1Data.state = APPSPI1_STATE_IDLE;
                }
            }
            break;
        }    
        case APPSPI1_STATE_ERROR:
        {
            SS1_Set();
            switch(appspi1Data.typeOfError)
            {
                case SPI1_OPEN_ERROR:
                case SPI1_SET_INSTANCE:    
                {
                    appspi1Data.adelay = RTC_Timer32CounterGet();
                    appspi1Data.state = APPSPI1_STATE_DELAY_TO_REOPEN_SPI;
                    break;
                }
                default:
                {
                    DRV_SPI_Close(appspi1Data.drvSPIHandle);
                    APPSPI1_Initialize();
                }    
            }        
            break;
        }
        case APPSPI1_STATE_DELAY_TO_REOPEN_SPI:
        {
            if ( abs_diff_uint32(RTC_Timer32CounterGet(), appspi1Data.adelay) > _500ms)
            {
                appspi1Data.state = APPSPI1_STATE_DRIVER_SETUP;
            }
            break;
        }    
        /* The default state should never be executed. */
        default: break;  /* TODO: Handle error in application's state machine. */
    }
}


/*******************************************************************************
 End of File
 */
