/******************************************************************************

@file  app_main.c

@brief This file contains the application main functionality

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-2024, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

******************************************************************************


*****************************************************************************/

//*****************************************************************************
//! Includes
//*****************************************************************************
#include "ti_ble_config.h"
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/menu_module/menu_module.h>
#include <app_main.h>
#include <ti/drivers/timer/LGPTimerLPF3.h>
#include <ti/drivers/timer/LGPTimerLPF3.h>
#include <ti/drivers/ADC.h>
#include <string.h>
#include <stdint.h>

//*****************************************************************************
//! Defines
//*****************************************************************************

//*****************************************************************************
//! Globals
extern void NotifyTrigger();
extern bStatus_t BLEAppUtil_advStart(uint8 handle, const BLEAppUtil_AdvStart_t *advStartInfo);
extern bStatus_t BLEAppUtil_advStop(uint8_t);
extern uint8_t broadcasterAdvHandle_1;
extern BLEAppUtil_AdvStart_t broadcasterStartAdvSet1;
ADC_Params ADCparams;
ADC_Handle adcHandle;
uint16_t value_gpio_level;
uint8_t counter = 0;
uint8_t device_name[5] = {0x04,0x42,0x42,0x43,0x44};
//*****************************************************************************

// Parameters that should be given as input to the BLEAppUtil_init function
BLEAppUtil_GeneralParams_t appMainParams =
{
    .taskPriority = 1,
    .taskStackSize = 1024,
    .profileRole = (BLEAppUtil_Profile_Roles_e)(HOST_CONFIG),
    .addressMode = DEFAULT_ADDRESS_MODE,
    .deviceNameAtt = attDeviceName,
    .pDeviceRandomAddress = pRandomAddress,
};

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ) )
BLEAppUtil_PeriCentParams_t appMainPeriCentParams =
{
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
 .connParamUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION,
#endif //#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
 .gapBondParams = &gapBondParams
};
#else //observer || broadcaster
BLEAppUtil_PeriCentParams_t appMainPeriCentParams;
#endif //#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ) )



//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      criticalErrorHandler
 *
 * @brief   Application task entry point
 *
 * @return  none
 */
void criticalErrorHandler(int32 errorCode , void* pInfo)
{
//    trace();
//
//#ifdef DEBUG_ERR_HANDLE
//
//    while (1);
//#else
//    SystemReset();
//#endif

}

/*********************************************************************
 * @fn      App_StackInitDone
 *
 * @brief   This function will be called when the BLE stack init is
 *          done.
 *          It should call the applications modules start functions.
 *
 * @return  none
 */
void App_StackInitDoneHandler(gapDeviceInitDoneEvent_t *deviceInitDoneData)
{
    bStatus_t status = SUCCESS;

    // Menu
    Menu_start();

    // Print the device ID address
    MenuModule_printf(APP_MENU_DEVICE_ADDRESS, 0, "BLE ID Address: "
                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_GREEN "%s" MENU_MODULE_COLOR_RESET,
                      BLEAppUtil_convertBdAddr2Str(deviceInitDoneData->devAddr));

    if ( appMainParams.addressMode > ADDRMODE_RANDOM)
    {
      // Print the RP address
        MenuModule_printf(APP_MENU_DEVICE_RP_ADDRESS, 0,
                     "BLE RP Address: "
                     MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_GREEN "%s" MENU_MODULE_COLOR_RESET,
                     BLEAppUtil_convertBdAddr2Str(GAP_GetDevAddress(FALSE)));
    }

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ) )
    status = DevInfo_start();
    if(status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
    status = SimpleGatt_start();
    if(status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
#endif

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ))  &&  defined(OAD_CFG)
    status =  OAD_start();
    if(status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
#endif

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
    // Any device that accepts the establishment of a link using
    // any of the connection establishment procedures referred to
    // as being in the Peripheral role.
    // A device operating in the Peripheral role will be in the
    // Peripheral role in the Link Layer Connection state.
    status = Peripheral_start();
    if(status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
#endif

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( BROADCASTER_CFG ) )
    // A device operating in the Broadcaster role is a device that
    // sends advertising events or periodic advertising events
    status = Broadcaster_start();
    if(status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
#endif

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )
    // A device that supports the Central role initiates the establishment
    // of an active physical link. A device operating in the Central role will
    // be in the Central role in the Link Layer Connection state.
    // A device operating in the Central role is referred to as a Central.
    status = Central_start();
    if(status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
#endif

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( OBSERVER_CFG ) )
    // A device operating in the Observer role is a device that
    // receives advertising events or periodic advertising events
    status = Observer_start();
    if(status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
#endif

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ) )
    status = Connection_start();
    if(status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
    status = Pairing_start();
    if(status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
    status = Data_start();
    if(status != SUCCESS)
    {
        // TODO: Call Error Handler
    }

#if defined (BLE_V41_FEATURES) && (BLE_V41_FEATURES & L2CAP_COC_CFG)
    /* L2CAP COC Init */
    status = L2CAPCOC_start();
    if ( status != SUCCESS )
    {
    // TODO: Call Error Handler
    }
#endif //(BLE_V41_FEATURES) && (BLE_V41_FEATURES & L2CAP_COC_CFG)

#endif
}

void timerCallback(LGPTimerLPF3_Handle lgptHandle, LGPTimerLPF3_IntMask interruptMask) {
   // interrupt callback code goes here. Minimize processing in interrupt.
   // BLEAppUtil_advStop(broadcasterAdvHandle_1);
    GPIO_toggle(CONFIG_GPIO_LED_RED);
    ADC_convert(adcHandle, &value_gpio_level);
    BLEAppUtil_invokeFunctionNoData(NotifyTrigger);
    //advData1[sizeof(advData1)-2] = counter++;
    //BLEAppUtil_advStart(peripheralAdvHandle_1, &advSetStartParamsSet_1);
    //BLEAppUtil_advStart(broadcasterAdvHandle_1, &broadcasterStartAdvSet1);
}

/*********************************************************************
 * @fn      appMain
 *
 * @brief   Application main function
 *
 * @return  none
 */
void appMain(void)
{
    LGPTimerLPF3_Handle lgptHandle;
    LGPTimerLPF3_Params params;
    uint32_t counterTarget;
    // Initialize parameters and assign callback function to be used
    LGPTimerLPF3_Params_init(&params);
    params.hwiCallbackFxn = timerCallback;
    params.prescalerDiv = 256;
    // Open driver
    lgptHandle = LGPTimerLPF3_open(0, &params);
    // Set counter target
    counterTarget = 4800000 - 1;  // 2560 ms with a system clock of 48 MHz
    LGPTimerLPF3_setInitialCounterTarget(lgptHandle, counterTarget, true);
    // Enable counter target interrupt
    LGPTimerLPF3_enableInterrupt(lgptHandle, LGPTimerLPF3_INT_TGT);
    // Start counter in count-up-periodic mode
    LGPTimerLPF3_start(lgptHandle, LGPTimerLPF3_CTL_MODE_UP_PER);
    // One-time init of ADC driver
    ADC_init();
    // initialize optional ADC parameters

    ADC_Params_init(&ADCparams);
    ADCparams.isProtected = true;
    // Open ADC channels for usage
    adcHandle = ADC_open(CONFIG_ADC_0, &ADCparams);
    // Call the BLEAppUtil module init function
    BLEAppUtil_init(&criticalErrorHandler, &App_StackInitDoneHandler,
                    &appMainParams, &appMainPeriCentParams);


    memset(scanResData1, '\0',sizeof(scanResData1));
    scanResData1[0] = device_name[0] + 1; //RamFlashData.device_name[0] + 1;
    scanResData1[1] = GAP_ADTYPE_LOCAL_NAME_COMPLETE;
    //    scanResData1[0] = 0x15;
    //    scanResData1[1] = GAP_ADTYPE_LOCAL_NAME_SHORT;
    for (uint8_t i = 1; i <= 4; i++)
    {
        scanResData1[i + 1] = device_name[i];
    }
    advData1[sizeof(advData1)-2] = 0x56;

}
