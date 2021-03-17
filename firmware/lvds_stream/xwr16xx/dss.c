/*
 *   @file  dss.c
 *
 *   @brief
 *      DSS Implementation of the Memory Capture
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2017 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** @defgroup DSS_XWR16_LVDS_STREAM_APP DSS LVDS Stream Application for XWR16xx
 *
 * ## Introduction #
 *
 * The DSS LVDS Stream application is only responsible for the execution
 * of the data path. It does not participate in any control activities.
 * The mmWave module is executed in ISOLATION mode on the MSS.
 *
 * Thus the DSS is responsible for loading of the LVDS Stream Profile
 * and the creation of the Test Framework Task on which context the
 * profile is executed.
 *
 * ## Configuration #
 *
 * As explained on the MSS; in the case of the User Buffer streaming. The
 * address of these buffers is known only to the DSS and not to the MSS.
 *
 * The DSS will be notified about this partial configuration and will
 * work to fix it. Once the configuration is fixed it will pass the
 * updated and correct configuration back to the profile via the
 * TestFmk_ioctl.
 *
 * ## Start #
 *
 * The DSS simply receives the finalize configuration report from the
 * MSS. It uses that configuration to start the profile and to be ready
 * for the ADC data to arrive.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c64p/Cache.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/drivers/adcbuf/ADCBuf.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/control/mmwavelink/include/rl_driver.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <ti/drivers/test/common/framework.h>
#include <ti/drivers/test/lvds_stream/lvds_stream.h>

/** @addtogroup DSS_XWR16_LVDS_STREAM_APP
 @{ */

/**************************************************************************
 *************************** Local Structures *****************************
 **************************************************************************/

/**
 * @brief
 *  Stream Demo Master Control Block
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  Stream Demo.
 */
typedef struct LVDSStream_MCB_t
{
    /**
     * @brief   SOC Handle:
     */
    SOC_Handle          socHandle;

    /**
     * @brief   Test Framework Handle:
     */
    TestFmk_Handle      fmkHandle;
}LVDSStream_MCB;

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/
static void LVDSStream_reportFxn
(
    TestFmk_Report  reportType,
    int32_t         errCode,
    uint32_t        arg0,
    uint32_t        arg1
);

static void LVDSStream_fmkTask(UArg arg0, UArg arg1);
static void LVDSStream_initTask(UArg arg0, UArg arg1);

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/**
 * @brief
 *  Global Variable for tracking information required by the Capture Demo
 */
LVDSStream_MCB    gLVDSStreamMCB;

/**
 * @brief   This is the SW Buffer1 which is streamed out if the CLI configuration
 * enables software streaming.
 */
#pragma DATA_SECTION(gSwBuffer1, ".swBuffer1");
#pragma DATA_ALIGN(gSwBuffer1, 8);
uint16_t gSwBuffer1[64];

/**
 * @brief   This is the SW Buffer2 which is streamed out if the CLI configuration
 * enables software streaming.
 */
#pragma DATA_SECTION(gSwBuffer2, ".swBuffer2");
#pragma DATA_ALIGN(gSwBuffer2, 8);
uint16_t gSwBuffer2[64];

/**************************************************************************
 ************************** LVDS Stream Functions *************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      This is the registered Test Framework Report function. This is invoked by the
 *      Test Framework to report the status
 *
 *  @param[in]  reportType
 *      Report Type
 *  @param[in]  errCode
 *      Error code associated with the status event.
 *  @param[in]  arg0
 *      Optional argument. This is status specific so please refer to the documentation
 *      about the status
 *  @param[in]  arg1
 *      Optional argument. This is status specific so please refer to the documentation
 *      about the status
 *
 *  @retval
 *      Not Applicable.
 */
static void LVDSStream_reportFxn
(
    TestFmk_Report  reportType,
    int32_t         errCode,
    uint32_t        arg0,
    uint32_t        arg1
)
{
    TestFmk_Cfg*        ptrTestFmkCfg;

    /* Process the Report Status:*/
    switch (reportType)
    {
        case TestFmk_Report_PROFILE_LOADED:
        {
            /* Was there an error reported? */
            if (errCode != 0)
            {
                /* Error: Unable to load the profile. */
                System_printf ("Error: Unable to load the profile [Error code %d]\n", errCode);
            }
            break;
        }
        case TestFmk_Report_PROFILE_OPENED:
        {
            /* Was there an error reported? */
            if (errCode == 0)
            {
                /* Success: The profile has been opened successfully. */
                System_printf ("Debug: Profile has been opened successfully\n");
            }
            else
            {
                /* Error: Unable to open the profile. */
                System_printf ("Error: Unable to open the profile [Error code %d]\n", errCode);
            }
            break;
        }
        case TestFmk_Report_PROFILE_CLOSED:
        {
            /* Was there an error reported? */
            if (errCode == 0)
            {
                /* Success: The profile has been closed successfully. */
                System_printf ("Debug: Profile has been closed successfully\n");
            }
            else
            {
                /* Error: Unable to close the profile. */
                System_printf ("Error: Unable to close the profile [Error code %d]\n", errCode);
            }
            break;
        }
        case TestFmk_Report_SET_CFG:
        {
            /* Was there any error reported? */
            if (errCode == 0)
            {
                /* Get the pointer to the test framework: */
                ptrTestFmkCfg = (TestFmk_Cfg*)arg0;
                DebugP_assert (ptrTestFmkCfg != NULL);

                /******************************************************************
                 * Start the Profile:
                 * - The report is an indication that the configuration has been
                 *   finalized and been passed to the profile successfully.
                 ******************************************************************/
                if (TestFmk_start (gLVDSStreamMCB.fmkHandle, &errCode) < 0)
                {
                    System_printf ("Error: Unable to start the profile [Error: %d]\n", errCode);
                    return;
                }

                /********************************************************************************
                 * The profile once started successfully will report its status to both the
                 * Instances. The mmWave sensor can only be started on the reception of this
                 * report. In the current use case the mmWave is executing in isolation mode only
                 * on the MSS; the sensor will be started there.
                 ********************************************************************************/
                break;
            }
            else
            {
                /* Error: Unable to get the configuration */
                System_printf ("Error: Unable to receive the configuration [Error code %d]\n", errCode);
            }
            break;
        }
        case TestFmk_Report_IOCTL:
        {
            /* Is this the user buffer configuration? */
            if (arg0 == LVDS_STREAM_PROFILE_SET_USERBUF_STREAM)
            {
                TestFmk_LVDSStreamProfileUserBufCfg*    ptrLVDSStreamProfileUserCfg;

                /* YES: Get the LVDS User Buffer configuration */
                ptrLVDSStreamProfileUserCfg = (TestFmk_LVDSStreamProfileUserBufCfg*)arg1;

                /* Do we need to *fix* the configuration? If we are executing in SW Mode the
                 * SW Buffers need to be populated by the DSS. */
                if (((ptrLVDSStreamProfileUserCfg->ptrUserBuffer1 == (uint8_t*)&gSwBuffer1[0]) &&
                     (ptrLVDSStreamProfileUserCfg->ptrUserBuffer2 == (uint8_t*)&gSwBuffer2[0]))
                                        ||
                    ((ptrLVDSStreamProfileUserCfg->ptrUserBuffer1 == NULL) &&
                     (ptrLVDSStreamProfileUserCfg->ptrUserBuffer2 == NULL)))
                {
                    /* NO: The configuration has already been fixed *or* there is no
                     * need to fix it. */
                }
                else
                {
                    /* YES: We need to fix the configuration because the MSS has populated
                     * the dummy configuration. So here we simply override it with the
                     * correct value. */
                    ptrLVDSStreamProfileUserCfg->ptrUserBuffer1  = (uint8_t*)&gSwBuffer1[0];
                    ptrLVDSStreamProfileUserCfg->sizeUserBuffer1 = sizeof(gSwBuffer1);
                    ptrLVDSStreamProfileUserCfg->ptrUserBuffer2  = (uint8_t*)&gSwBuffer2[0];
                    ptrLVDSStreamProfileUserCfg->sizeUserBuffer2 = sizeof(gSwBuffer2);

                    /* Pass this new updated configuration to the framework:*/
                    if (TestFmk_ioctl (gLVDSStreamMCB.fmkHandle,
                                       LVDS_STREAM_PROFILE_SET_USERBUF_STREAM,
                                       ptrLVDSStreamProfileUserCfg,
                                       sizeof (TestFmk_LVDSStreamProfileUserBufCfg),
                                       &errCode) < 0)
                    {
                        System_printf ("Error: Unable to *fix* the user buffer [Error: %d]\n", errCode);
                        break;
                    }

                    /* The configuration has been fixed. The MSS can now finalize the configuration. */
                }
            }
            break;
        }
        case TestFmk_Report_PROFILE_STARTED:
        {
            break;
        }
        case TestFmk_Report_PROFILE_STOPPED:
        {
            /* Was there an error reported? */
            if (errCode == 0)
            {
                /* Success: The profile has been closed successfully. */
                System_printf ("Debug: Profile has been stopped successfully\n");
            }
            else
            {
                /* Error: Unable to close the profile. */
                System_printf ("Error: Unable to stop the profile [Error code %d]\n", errCode);
            }
            break;
        }
        case TestFmk_Report_PROFILE_UNLOADED:
        {
            /* Was there an error reported? */
            if (errCode == 0)
            {
                /* Success: The profile has been unloaded successfully. */
                System_printf ("Debug: Profile has been unloaded successfully\n");
            }
            else
            {
                /* Error: Unable to unload the profile. */
                System_printf ("Error: Unable to unload the profile [Error code %d]\n", errCode);
            }
            break;
        }
        default:
        {
            DebugP_assert (0);
            break;
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The task is used to provide an execution context for the Test Framework
 *
 *  @retval
 *      Not Applicable.
 */
static void LVDSStream_fmkTask(UArg arg0, UArg arg1)
{
    int32_t             errCode;
    TestFmk_Result      result;

    while (1)
    {
        /* Execute the Test Framework module: */
        if (TestFmk_execute (gLVDSStreamMCB.fmkHandle, &result, &errCode) < 0)
            System_printf ("Error: Test Framework execution failed [Error code %d]\n", errCode);
    }
}

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
static void LVDSStream_initTask(UArg arg0, UArg arg1)
{
    Task_Params           taskParams;
    int32_t               errCode;
    TestFmk_InitCfg       fmkInitCfg;

    /* Debug Message: */
    System_printf("Debug: Launched the Demo Initialization Task\n");

    /*****************************************************************************
     * Initialize the mmWave SDK components:
     *****************************************************************************/

    /* Initialize the Mailbox */
    Mailbox_init(MAILBOX_TYPE_DSS);

    /* Initialize the Cycle Profiler: */
    Cycleprofiler_init();

    /*****************************************************************************
     * Initialization of the User Buffers:
     * - The bootloader cannot initialize and load the HSRAM so we simply
     *   initialize the buffers here.
     *****************************************************************************/
    memset ((void*)&gSwBuffer1[0], 0, sizeof(gSwBuffer1));
    memset ((void*)&gSwBuffer2[0], 0, sizeof(gSwBuffer2));

    /* Populate the SW Buffer1: */
    gSwBuffer1[0]  = 200U;
    gSwBuffer1[1]  = 201U;
    gSwBuffer1[2]  = 202U;
    gSwBuffer1[3]  = 203U;
    gSwBuffer1[4]  = 204U;
    gSwBuffer1[5]  = 205U;
    gSwBuffer1[6]  = 206U;
    gSwBuffer1[7]  = 207U;
    gSwBuffer1[8]  = 208U;
    gSwBuffer1[9]  = 209U;
    gSwBuffer1[10] = 210U;
    gSwBuffer1[11] = 211U;

    /* Populate the SW Buffer2: */
    gSwBuffer2[0]  = 300U;
    gSwBuffer2[1]  = 301U;
    gSwBuffer2[2]  = 302U;
    gSwBuffer2[3]  = 303U;
    gSwBuffer2[4]  = 304U;
    gSwBuffer2[5]  = 305U;
    gSwBuffer2[6]  = 306U;
    gSwBuffer2[7]  = 307U;
    gSwBuffer2[8]  = 308U;
    gSwBuffer2[9]  = 309U;
    gSwBuffer2[10] = 310U;
    gSwBuffer2[11] = 311U;

    /*****************************************************************************
     * Initialization of the Test Framework:
     *****************************************************************************/
    memset ((void *)&fmkInitCfg, 0, sizeof(TestFmk_InitCfg));

    /* Setup the configuration: */
    fmkInitCfg.socHandle  = gLVDSStreamMCB.socHandle;
    fmkInitCfg.ctrlHandle = NULL;
    fmkInitCfg.reportFxn  = LVDSStream_reportFxn;

    /* Initialize the Test Framework: */
    gLVDSStreamMCB.fmkHandle = TestFmk_init (&fmkInitCfg, &errCode);
    if (gLVDSStreamMCB.fmkHandle == NULL)
    {
        System_printf ("Error: Unable to initialize the Test Framework [Error: %d]\n", errCode);
        return;
    }
    System_printf ("Debug: Test Framework Initialized\n");

    /* Synchronization: This will synchronize the execution of the control module
     * between the domains. This is a prerequiste and always needs to be invoked. */
    while (1)
    {
        int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = TestFmk_synch (gLVDSStreamMCB.fmkHandle, &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the framework */
            System_printf ("Error: Framework Synchronization failed [Error code %d]\n", errCode);
            return;
        }
        if (syncStatus == 1)
        {
            /* Synchronization acheived: */
            break;
        }
        /* Sleep and poll again: */
        Task_sleep(1);
    }
    System_printf("Debug: Framework Sync was successful\n");

    /* Launch the Test Framework Task:
     * - This is executed as the highest priority in the core. */
    Task_Params_init(&taskParams);
    taskParams.priority  = 4;
    taskParams.stackSize = 4*1024;
    Task_create(LVDSStream_fmkTask, &taskParams, NULL);

    /*************************************************************************
     * The Framework is operational and we can now load the profile
     *************************************************************************/
    if (TestFmk_loadProfile (gLVDSStreamMCB.fmkHandle, &gLVDSStreamProfileCfg, &errCode) < 0)
    {
        System_printf ("Error: Unable to load the profile [Error: %d]\n", errCode);
        return;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      Entry point into the CAPTURE Demo on DSS. The demo can run as ISOLATED
 *  or COOPERATIVE mode.
 *
 *  @retval
 *      Not Applicable.
 */
int main (void)
{
    Task_Params	    taskParams;
    int32_t         errCode;
    SOC_Cfg         socCfg;

    /* Initialize the Streaming Demo MCB */
    memset ((void*)&gLVDSStreamMCB, 0, sizeof(LVDSStream_MCB));

    /* Initialize the SOC confiugration: */
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_INIT;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    gLVDSStreamMCB.socHandle = SOC_init (&socCfg, &errCode);
    if (gLVDSStreamMCB.socHandle == NULL)
    {
        System_printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        return 0;
    }

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    Task_create(LVDSStream_initTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
    return 0;
}

/**
@}
*/

