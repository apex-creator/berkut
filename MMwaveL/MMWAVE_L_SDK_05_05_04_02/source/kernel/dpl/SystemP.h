/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

#ifndef SYSTEMP_H
#define SYSTEMP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <kernel/dpl/DebugP.h>

/**
 * \defgroup KERNEL_DPL_SYSTEM  APIs for system level define's and function's
 * \ingroup KERNEL_DPL
 *
 * @{
 */

/** @name Return status
 */
/**@{*/
/**
 * \brief Return status when the API execution was successful
 */
#define SystemP_SUCCESS   ((int32_t )0)

/**
 * \brief Return status when the API execution was not successful due to a failure
 */
#define SystemP_FAILURE   ((int32_t)-1)

/**
 * \brief Return status when the API execution failed due to a time out
 */
#define SystemP_TIMEOUT   ((int32_t)-2)

/**
 * \brief Return status when the API execution failed due invalid parameters
 */
#define SystemP_INVALID_PARAM   ((int32_t)-3)

/**
 * \brief Return status when the API execution failed due to driver busy
 */
#define SystemP_BUSY   ((int32_t)-4)

/**
 * \brief Return status when the API execution failed due to invalid LLD state
 */
#define SystemP_INVALID_STATE  ((int32_t)-5)

/**
 * \brief Return status when the API execution failed due to lack of resources (memory / HW)
 */
#define SystemP_OUT_OF_RESOURCES  ((int32_t)-6)

/**@}*/

/** @name Timeout values
 *  @anchor SystemP_Timeout
 */
/**@{*/

/**
 * \brief Value to use when needing a timeout of zero or NO timeout, return immediately on resource not available.
 */
#define SystemP_NO_WAIT        ((uint32_t)0)

/**
 * \brief Value to use when needing a timeout of infinity or wait forver until resource is available
 */
#define SystemP_WAIT_FOREVER   ((uint32_t)-1)
/**@}*/

/** @} */


#define LLD_STATE_RESET (0U)
#define LLD_STATE_READY (1U)
#define LLD_STATE_BUSY  (2U)
#define LLD_STATE_ERROR (3U)


#define LLD_PARAM_CHECK_ENABLED
#ifdef LLD_PARAM_CHECK_ENABLED
#ifdef LLD_PARAM_CHECK_DEBUG_ASSERT
static inline void _Param_check_lld(int expression, const char *file, const char *function, int line, const char *expressionString);

static inline void _Param_check_lld(int expression, const char *file, const char *function, int line, const char *expressionString)
{

    _DebugP_assert(expression, file, function, line, expressionString);
}
#define LLD_PARAMS_CHECK(expression) _Param_check_lld(expression, \
            __FILE__, __FUNCTION__, __LINE__, #expression)
#else
#define LLD_PARAMS_CHECK(expression) \
if (status == SystemP_SUCCESS) { \
    if(!(expression)) { \
        status = SystemP_INVALID_PARAM; \
    } \
}
#endif /*! LLD_PARAM_CHECK_DEBUG_ASSERT */
#else
#define LLD_PARAMS_CHECK(expression)
#endif /*! LLD_PARAM_CHECK_ENABLED */

/** @name LLD (Low Level Driver) states
 */
/**@{*/
/**
 * \brief LLD is in Reset State prior to driver init and post driver deinit
 */
#define LLD_STATE_RESET (0U)

/**
 * \brief LLD accepts runtime APIs only Ready State, otherwise return error
 */
#define LLD_STATE_READY (1U)

/**
 * \brief LLD is busy performing operation with peripherals, return error when APIs are invoked
 */
#define LLD_STATE_BUSY  (2U)

/**
 * \brief LLD ran into error, returns error for all APIs other than deinit in this state
 */
#define LLD_STATE_ERROR (3U)

/**@}*/

/**
 * \brief NOT_IN_USE macro to highlight unused parameters
 */
#define NOT_IN_USE(x) (void) 0

#ifdef __cplusplus
}
#endif

#endif /* SYSTEMP_H */

/**
 * \defgroup KERNEL_DPL APIs for Driver Porting Layer
 *
 * This module contains APIs which are used by the drivers to make them agnostic of the underlying OS and CPU architecture.
 */

/**
 * \defgroup DRV_MODULE APIs for SOC Specific Device Drivers
 *
 * This module contains APIs for device drivers for various peripherals supported in this SDK
 */

/**
 * \defgroup BOARD_MODULE APIs for Board Specific Device Drivers
 *
 * This module contains APIs for device drivers for various peripherals supported on the EVM or board supported by this SOC
 */
