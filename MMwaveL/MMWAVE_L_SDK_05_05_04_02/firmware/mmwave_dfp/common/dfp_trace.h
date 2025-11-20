/*!*****************************************************************************
 * @file dfp_trace.h
 *
 * @brief DFP debug trace header file.
 *
 * @b Description @n
 * This is a DFP mmWaveLink/FECSS library debug trace support header file defines
 * Debug assert and print MACROs.
 *
 *
 * @note
 * <B> © Copyright 2022, Texas Instruments Incorporated – www.ti.com </B>
 * @n
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * @n
 * <ul>
 *   <li> Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 * @n
 *   <li> Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 * @n
 *   <li> Neither the name of Texas Instruments Incorporated nor the names of
 *        its contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
 * </ul>
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS
 * IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES LOSS OF USE,
 * DATA, OR PROFITS OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @addtogroup MMWAVE_DFP_DEBUG mmWave DFP Debug Module
 *  @{
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Revision History   :
 *------------------------------------------------------------------------------
 * Version Date        Author  Ticket No           Description
 *------------------------------------------------------------------------------
 * 0.1     14Jan2021   TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * FILE INCLUSION PROTECTION
 *******************************************************************************
 */
#ifndef DFP_TRACE_H
#define DFP_TRACE_H

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */
#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * MACRO DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name DFP Debug module trace Driver MACROs
 * @{
 */

/*!
 * DFP Debug Logger print macros
 */
#if M_DFP_DISABLE_LOGGING

#define M_DFP_LOG_ERR_ARG0(formatString)
#define M_DFP_LOG_ERR_ARG1(formatString,varArg1)
#define M_DFP_LOG_ERR_ARG2(formatString,varArg1,varArg2)
#define M_DFP_LOG_ERR_ARG3(formatString,varArg1,varArg2,varArg3)

#define M_DFP_LOG_INFO_ARG0(formatString)
#define M_DFP_LOG_INFO_ARG1(formatString,varArg1)
#define M_DFP_LOG_INFO_ARG2(formatString,varArg1,varArg2)
#define M_DFP_LOG_INFO_ARG3(formatString,varArg1,varArg2,varArg3)

#define M_DFP_LOG_ADDR_DATA(arg1,arg2)

#else
#define M_DFP_LOG_ERR_ARG0(formatString) \
    {\
        /*! get the debug logger callback function pointer  */\
        T_DFP_PRINT_FUNCP p_fPtr = fe_getDbgLogFptr();\
        /*! Check for NULL Pointer */\
        if (p_fPtr != M_NULL_PTR)\
        {\
            /*! Log the data with function name, line no and passed arg */\
            (void)p_fPtr("[DFP_ERROR] %s:%d::" formatString"\n", __FUNCTION__, __LINE__);\
        }\
    }

#define M_DFP_LOG_ERR_ARG1(formatString, varArg1) \
    {\
        /*! get the debug logger callback function pointer  */\
        T_DFP_PRINT_FUNCP p_fPtr = fe_getDbgLogFptr();\
        /*! Check for NULL Pointer */\
        if (p_fPtr != M_NULL_PTR)\
        {\
            /*! Log the data with function name, line no and passed arg */\
            (void)p_fPtr("[DFP_ERROR] %s:%d::" formatString"\n", __FUNCTION__, __LINE__, (varArg1));\
        }\
    }

#define M_DFP_LOG_ERR_ARG2(formatString, varArg1, varArg2) \
    {\
        /*! get the debug logger callback function pointer  */\
        T_DFP_PRINT_FUNCP p_fPtr = fe_getDbgLogFptr();\
        /*! Check for NULL Pointer */\
        if (p_fPtr != M_NULL_PTR)\
        {\
            /*! Log the data with function name, line no and passed arg */\
            (void)p_fPtr("[DFP_ERROR] %s:%d::" formatString"\n", __FUNCTION__, __LINE__, (varArg1), (varArg2));\
        }\
    }

#define M_DFP_LOG_ERR_ARG3(formatString, varArg1, varArg2, varArg3) \
    {\
        /*! get the debug logger callback function pointer  */\
        T_DFP_PRINT_FUNCP p_fPtr = fe_getDbgLogFptr();\
        /*! Check for NULL Pointer */\
        if (p_fPtr != M_NULL_PTR)\
        {\
            /*! Log the data with function name, line no and passed arg */\
            (void)p_fPtr("[DFP_ERROR] %s:%d::" formatString"\n", __FUNCTION__, __LINE__, (varArg1), (varArg2), (varArg3));\
        }\
    }

#define M_DFP_LOG_INFO_ARG0(formatString) \
    {\
        /*! get the debug logger callback function pointer  */\
        T_DFP_PRINT_FUNCP p_fPtr = fe_getDbgLogFptr();\
        /*! Check for NULL Pointer */\
        if (p_fPtr != M_NULL_PTR)\
        {\
            /*! Log the data with function name, line no and passed arg */\
            (void)p_fPtr("[DFP_INFO]::" formatString"\n");\
        }\
    }

#define M_DFP_LOG_INFO_ARG1(formatString, varArg1) \
    {\
        /*! get the debug logger callback function pointer  */\
        T_DFP_PRINT_FUNCP p_fPtr = fe_getDbgLogFptr();\
        /*! Check for NULL Pointer */\
        if (p_fPtr != M_NULL_PTR)\
        {\
            /*! Log the data with function name, line no and passed arg */\
            (void)p_fPtr("[DFP_INFO]::" formatString"\n", (varArg1));\
        }\
    }

#define M_DFP_LOG_INFO_ARG2(formatString, varArg1, varArg2) \
    {\
        /*! get the debug logger callback function pointer  */\
        T_DFP_PRINT_FUNCP p_fPtr = fe_getDbgLogFptr();\
        /*! Check for NULL Pointer */\
        if (p_fPtr != M_NULL_PTR)\
        {\
            /*! Log the data with function name, line no and passed arg */\
            (void)p_fPtr("[DFP_INFO]::" formatString"\n", (varArg1), (varArg2));\
        }\
    }

#define M_DFP_LOG_INFO_ARG3(formatString, varArg1, varArg2, varArg3) \
    {\
        /*! get the debug logger callback function pointer  */\
        T_DFP_PRINT_FUNCP p_fPtr = fe_getDbgLogFptr();\
        /*! Check for NULL Pointer */\
        if (p_fPtr != M_NULL_PTR)\
        {\
            /*! Log the data with function name, line no and passed arg */\
            (void)p_fPtr("[DFP_INFO]::" formatString"\n", (varArg1), (varArg2), (varArg3));\
        }\
    }

#define M_DFP_LOG_ADDR_DATA(arg1, arg2) \
    {\
        /*! get the debug logger callback function pointer  */\
        T_DFP_PRINT_FUNCP p_fPtr = fe_getDbgLogFptr();\
        /*! Check for NULL Pointer */\
        if (p_fPtr != M_NULL_PTR)\
        {\
            /*! Log the data with function name, line no and passed arg */\
            (void)p_fPtr("[DFP_FECSS_ADDR_DATA]::%x:%x \n", (UINT32)(arg1), (UINT32)(arg2));\
        }\
    }

#endif

/*!
 * DFP Debug Logger ASSERT macro
 */
#define M_DFP_ASSERT(expr, xw_return, xw_errorCode) \
    {\
        /*! If exected expression fails then print error and return error code */\
        if ((expr) != M_LOGICAL_TRUE)\
        {\
            M_DFP_LOG_ERR_ARG1("DFP_FATAL_ERROR:%d \n", xw_errorCode);\
            /*! - Update error code  */\
            xw_return = xw_errorCode; \
        }\
        else\
        {\
            /*! - no Error if exected expression is true  */\
            xw_return = M_DFP_RET_CODE_OK;\
        }\
    }

/*! @} */
/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */

/*******************************************************************************
 * EXTERN GLOBAL VARIABLES/DATA-TYPES DECLARATIONS
 *******************************************************************************
 */

/*******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************
 */
/*!
 * @name DFP Debug module logger driver functions
 * @{
 */
M_LIB_EXPORT T_DFP_PRINT_FUNCP fe_getDbgLogFptr_rom(void);


/*! @} */
#ifdef __cplusplus
}
#endif

/* End of group */
/*! @} */
#endif
/*
 * END OF fe_driver.h
 */


