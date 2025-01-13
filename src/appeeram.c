/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    appeeram.c

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

#include "appeeram.h"
#include "definitions.h"
#include <string.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
#define CRC_SEED                            0xFFFFFFFF
#define MAXIMUM_TEMPERATURE_PER_POINT       250
#define MAXIMUM_NUMBER_EERAM_READ_ATTEMPTS  0x05

#define MAXIMUM_HEAT_TIME                   150
#define MINIMUM_HEAT_TIME                   50

#define MAXIMUM_FLUX_ACTIVATION_TIME        150
#define MINIMUM_FLUX_ACTIVATION_TIME        50

#define MAXIMUM_REFLOW_TIME                 100
#define MINIMUM_REFLOW_TIME                 20

#define MAXIMUM_COOLING_TIME                100
#define MINIMUM_COOLING_TIME                20
// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APPEERAM_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APPEERAM_DATA appeeramData;
uint8_t BufferTransmission[MAXIMUM_BUFFER_TRANSMISSION];
uint8_t BufferReception[MAXIMUM_BUFFER_RECEIVE];
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
void initializeParametersI2C2(void);
void APPEERAM_I2C_EventHandler (DRV_I2C_TRANSFER_EVENT event,DRV_I2C_TRANSFER_HANDLE transferHandle,uintptr_t context);
APPEERAM_STATES analyzeNumberAttempts(void);
bool verifyDataStoredInEERAM(void);
/* TODO:  Add any necessary local functions.
*/
void initializeParametersI2C2(void)
{
    appeeramData.drvI2CHandle   = DRV_HANDLE_INVALID;
    appeeramData.transferHandle = DRV_I2C_TRANSFER_HANDLE_INVALID;
    //appeeramData.transferStatus = APPEERAM_TRANSFER_STATUS_ERROR;
    appeeramData.attempts = 0x00;
    appeeramData.typeOfError = I2C2_NO_ERROR;
    appeeramData.adelay = RTC_Timer32CounterGet();
}
void APPEERAM_I2C_EventHandler (DRV_I2C_TRANSFER_EVENT event,DRV_I2C_TRANSFER_HANDLE transferHandle,uintptr_t context)
{
    APPEERAM_TRANSFER_STATUS* transferStatus = (APPEERAM_TRANSFER_STATUS*)context;
    if (event == DRV_I2C_TRANSFER_EVENT_COMPLETE)
    {
        if (transferStatus)
        {
            *transferStatus = APPEERAM_TRANSFER_STATUS_SUCCESS;
        }
    }
    else
    {
        if (transferStatus)
        {
            *transferStatus = APPEERAM_TRANSFER_STATUS_ERROR;
        }
    }
}
APPEERAM_STATES analyzeNumberAttempts(void)
{
    appeeramData.attempts++;
    if (appeeramData.attempts > MAXIMUM_NUMBER_EERAM_READ_ATTEMPTS)
    {
        appeeramData.attempts = 0x00;
        appeeramData.errorAPPEERAM = I2C2_EERAM_ILLOLOGICAL_DATA_ERROR; 
        return APPEERAM_STATE_DEFAULT_VALUES;
    }
    else
    {
        appeeramData.adelay = RTC_Timer32CounterGet();
        return APPEERAM_STATUS_WRITE_INITIAL_ADDRESS_READ;
    }
}

bool verifyDataStoredInEERAM(void)
{
    uint16_t time;
    /* Parameters of point A of the temperature curve */
    if (BufferReception[0x00] > MAXIMUM_TEMPERATURE_PER_POINT)
    {
        return false;
    }
    time = (uint16_t)BufferReception[0x01] | ((uint16_t)BufferReception[0x02] << 8);
    if (time > MAXIMUM_HEAT_TIME || time < MINIMUM_HEAT_TIME)
    {
        return false;
    }
    /*Parameters of point B of the temperature curve*/ 
    if (BufferReception[0x03] > MAXIMUM_TEMPERATURE_PER_POINT)
    {
        return false;
    }
    time = (uint16_t)BufferReception[0x04] | ((uint16_t)BufferReception[0x05] << 8);
    if (time > MAXIMUM_FLUX_ACTIVATION_TIME || time < MINIMUM_FLUX_ACTIVATION_TIME)
    {
        return false;
    }
    /*Parameters of point C of the temperature curve*/ 
    if (BufferReception[0x06] > MAXIMUM_TEMPERATURE_PER_POINT)
    {
        return false;
    }
    time = (uint16_t)BufferReception[0x07] | ((uint16_t)BufferReception[0x08] << 8);
    if (time > MAXIMUM_REFLOW_TIME || time < MINIMUM_REFLOW_TIME )
    {
        return false;
    }
     /*Parameters of point D of the temperature curve*/ 
    if (BufferReception[0x09] > MAXIMUM_TEMPERATURE_PER_POINT)
    {
        return false;
    }
    time = (uint16_t)BufferReception[0x0A] | ((uint16_t)BufferReception[0x0B] << 8);
    if (time > MAXIMUM_COOLING_TIME || time < MINIMUM_COOLING_TIME)
    {
        return false;
    }
    float k;
    memcpy(&k, &BufferReception[0x0C], sizeof(float)); //Kp
    if (k < 0)
    {
        return false;
    }
    memcpy(&k, &BufferReception[0x10], sizeof(float)); //Ki    
    if (k < 0)
    {
        return false;
    }
    memcpy(&k, &BufferReception[0x14], sizeof(float)); //Kd   
     if (k < 0)
    {
        return false;
    }
    return true;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APPEERAM_Initialize ( void )

  Remarks:
    See prototype in appeeram.h.
 */

void APPEERAM_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appeeramData.state = APPEERAM_STATE_INIT;
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    initializeParametersI2C2();
}


/******************************************************************************
  Function:
    void APPEERAM_Tasks ( void )

  Remarks:
    See prototype in appeeram.h.
 */

void APPEERAM_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appeeramData.state )
    {
        /* Application's initial state. */
        case APPEERAM_STATE_INIT:
        {
            if ( abs_diff_uint32(RTC_Timer32CounterGet(), appeeramData.adelay) > _25ms)
            {
                appeeramData.state = APPEERAM_STATE_HAS_ELAPSED_25ms;
            }
            break;
        }
        case APPEERAM_STATE_HAS_ELAPSED_25ms:
        {
            appeeramData.drvI2CHandle = DRV_I2C_Open( DRV_I2C_INDEX_0, DRV_IO_INTENT_READWRITE );
            if(DRV_HANDLE_INVALID == appeeramData.drvI2CHandle)
            {
                appeeramData.typeOfError = I2C2_OPEN_ERROR;
                appeeramData.state = APPEERAM_STATE_ERROR;
            }
            else
            {
                DRV_I2C_TransferEventHandlerSet(appeeramData.drvI2CHandle,APPEERAM_I2C_EventHandler,(uintptr_t)&appeeramData.transferStatus);/* Register the I2C Driver event Handler */
                appeeramData.state  = APPEERAM_STATE_READ_BIT_ASE;
            }
            break;
        }
        /* TODO: implement your application state machine.*/
        case APPEERAM_STATE_READ_BIT_ASE:
        {
            appeeramData.transferStatus = APPEERAM_TRANSFER_STATUS_IN_PROGRESS;/* Add a dummy write transfer request to verify whether EEPROM is ready */
            DRV_I2C_ReadTransferAdd(appeeramData.drvI2CHandle,EERAMStatusByte>>1,(void *)&BufferReception[0],0x01,&appeeramData.transferHandle);
            if(DRV_I2C_TRANSFER_HANDLE_INVALID ==  appeeramData.transferHandle)
            {
                appeeramData.typeOfError = I2C2_ASSIGN_READ_HANDLER_ASE_ERROR;
                appeeramData.state = APPEERAM_STATE_ERROR;
            }
            else
            {
                appeeramData.state = APPEERAM_STATE_ANALYZE_BIT_ASE;
            }
            break;
        }
        case APPEERAM_STATE_ANALYZE_BIT_ASE:
        {
            if (appeeramData.transferStatus == APPEERAM_TRANSFER_STATUS_SUCCESS)
            {
                if (0x02 == (BufferReception[0x00] & 0x02))
                {
                    appeeramData.adelay = RTC_Timer32CounterGet();
                    appeeramData.state = APPEERAM_STATUS_WRITE_INITIAL_ADDRESS_READ;
                }
                else
                {
                    appeeramData.state = APPEERAM_STATUS_ACTIVATE_BIT_ASE;
                }
            }
            else if (appeeramData.transferStatus == APPEERAM_TRANSFER_STATUS_ERROR)
            {
                appeeramData.typeOfError = I2C2_COMPLETE_READ_TRANSFER_ASE_ERROR;
                appeeramData.state = APPEERAM_STATE_ERROR;
            }
            break;
        }
        case APPEERAM_STATUS_ACTIVATE_BIT_ASE:
        {
            appeeramData.transferStatus = APPEERAM_TRANSFER_STATUS_IN_PROGRESS;
            BufferTransmission[0x00] = StatusByteAddress; //address 
            BufferTransmission[0x01] = StatusByteValue; //value to write 2
            DRV_I2C_WriteTransferAdd(appeeramData.drvI2CHandle, EERAMStatusByte>>1,(void *)&BufferTransmission[0x00],0x02,&appeeramData.transferHandle);
            if(DRV_I2C_TRANSFER_HANDLE_INVALID ==  appeeramData.transferHandle)
            {
                appeeramData.typeOfError = I2C2_ASSIGN_WRITE_HANDLER_ASE_ERROR;
                appeeramData.state = APPEERAM_STATE_ERROR;
            }
            else
            {
                appeeramData.state = APPEERAM_STATUS_WAIT_WRITE_BIT_ASE;
            }
            break;
        }
        case APPEERAM_STATUS_WAIT_WRITE_BIT_ASE:
        {
             if (appeeramData.transferStatus == APPEERAM_TRANSFER_STATUS_SUCCESS)
            {
                appeeramData.state = APPEERAM_STATUS_WRITE_INITIAL_ADDRESS_READ;
                appeeramData.adelay = RTC_Timer32CounterGet();
            }
            else if (appeeramData.transferStatus == APPEERAM_TRANSFER_STATUS_ERROR)
            {
                appeeramData.typeOfError = I2C2_COMPLETE_WRITE_TRANSFER_ASE_ERROR;
                appeeramData.state = APPEERAM_STATE_ERROR;
            }
            break;
        }
        case APPEERAM_STATUS_WRITE_INITIAL_ADDRESS_READ:
        {
            if ( abs_diff_uint32(RTC_Timer32CounterGet(), appeeramData.adelay) > _31ms)
            {
                appeeramData.transferStatus = APPEERAM_TRANSFER_STATUS_IN_PROGRESS;
                BufferTransmission[0x00] = 0x00; //High address inside the eeram 
                BufferTransmission[0x01] = 0x00; //Low direction inside the eeram 
                DRV_I2C_WriteTransferAdd(appeeramData.drvI2CHandle,ByteControlWriteEERAM>>1,(void *)&BufferTransmission[0x00],0x02,&appeeramData.transferHandle);
                if(DRV_I2C_TRANSFER_HANDLE_INVALID ==  appeeramData.transferHandle)
                {
                    appeeramData.typeOfError = I2C2_ASSIGN_WRITE_HANDLER_INITIAL_ADDRESS_ERROR;
                    appeeramData.state = APPEERAM_STATE_ERROR;
                }
                else
                {
                    appeeramData.state = APPEERAM_STATUS_READ_ARRAY_EERAM;
                }
            }
            break;
        }
        case APPEERAM_STATUS_READ_ARRAY_EERAM:
        {
            if (appeeramData.transferStatus == APPEERAM_TRANSFER_STATUS_SUCCESS)
            {
                appeeramData.transferStatus = APPEERAM_TRANSFER_STATUS_IN_PROGRESS;
                appeeramData.stateWhereToJumpAfterCRC = APPEERAM_STATUS_ANALYZE_DATA_READ_FROM_EERAM;
                DRV_I2C_ReadTransferAdd(appeeramData.drvI2CHandle,ByteControlReadEERAM>>1,(void *)&BufferReception[0],APPEERAM_NUMBER_BYTE_READ + 0x02,&appeeramData.transferHandle);
                if(DRV_I2C_TRANSFER_HANDLE_INVALID ==  appeeramData.transferHandle)
                {
                    appeeramData.typeOfError = I2C2_ASSIGN_READ_HANDLER_EERAM_ARRAY_ERROR;
                    appeeramData.state = APPEERAM_STATE_ERROR;
                }
                else
                {
                    appeeramData.state = APPEERAM_STATE_INITIALIAR_CRC;
                }
            }
            else if (appeeramData.transferStatus == APPEERAM_TRANSFER_STATUS_ERROR)
            {
                appeeramData.typeOfError = I2C2_COMPLETE_WRITE_TRANSFER_INITIAL_ADDRESS_ERROR;
                appeeramData.state = APPEERAM_STATE_ERROR;
            }
            break;
        }
        case APPEERAM_STATE_INITIALIAR_CRC:
        {
            DSU_REGS->DSU_ADDR = (uint32_t)&BufferReception;
            DSU_REGS->DSU_LENGTH = (uint32_t)APPEERAM_NUMBER_BYTE_READ;
            /* Initial CRC Value  */
            DSU_REGS->DSU_DATA = CRC_SEED;
            /* Clear Status Register */
            DSU_REGS->DSU_STATUSA = DSU_REGS->DSU_STATUSA;
            DSU_REGS->DSU_CTRL = DSU_CTRL_CRC_Msk;
            appeeramData.state = APPEERAM_STATUS_WAIT_CRC;
            break;
        }
        case APPEERAM_STATUS_WAIT_CRC:
        {
            if (DSU_REGS->DSU_STATUSA & DSU_STATUSA_DONE_Msk)
            {
                if((DSU_REGS->DSU_STATUSA & DSU_STATUSA_BERR_Msk) == 0U )
                {
                    uint32_t crcDeEERAM= (uint32_t)(BufferReception[APPEERAM_NUMBER_BYTE_READ -0x02] | (uint32_t)(BufferReception[APPEERAM_NUMBER_BYTE_READ -0x01] << 8));
                    if (crcDeEERAM == DSU_REGS->DSU_DATA)
                    {
                        appeeramData.state = appeeramData.stateWhereToJumpAfterCRC;
                    }
                    else
                    {
                        appeeramData.state = analyzeNumberAttempts();
                    }
                }
                else
                {
                    appeeramData.state = APPEERAM_STATE_INITIALIAR_CRC;
                }
            }
            break;
        }
        case APPEERAM_STATUS_ANALYZE_DATA_READ_FROM_EERAM:
        {
            if (verifyDataStoredInEERAM())
            {
                appeeramData.state = APPEERAM_STATE_IDLE;
            }
            else
            {
                appeeramData.state = APPEERAM_STATE_DEFAULT_VALUES;
            }
            break;
        }
        case APPEERAM_STATE_DEFAULT_VALUES:
        {
            
            break;
        }
        case APPEERAM_STATE_IDLE: break;
        case APPEERAM_STATE_ERROR:
        {
            
            break;
        }
        /* The default state should never be executed. */
        default: break; /* TODO: Handle error in application's state machine. */
    }
}


/*******************************************************************************
 End of File
 */
