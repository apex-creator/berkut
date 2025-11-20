/**
 *   @file  mmwave_fullcfg.c
 *
 *   @brief
 *      The file implements the functions which are required to support
 *      the FULL configuration mode.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016-2021 Texas Instruments, Inc.
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#include <stdint.h>
#include <string.h>

/* Includes from MCU Plus SDK */
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HeapP.h>

/* Includes from Mmwave SDK */
#include <control/mmwave/mmwave.h>
#include <control/mmwave/include/mmwave_internal.h>

/* User defined heap memory and handle */
#define MMWAVE_HEAP_MEM_SIZE  (16*1024u)

static uint8_t gMmwHeapMem[MMWAVE_HEAP_MEM_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));
static HeapP_Object gMmwHeapObj;

static bool isHeapMemAllocated = false;

/**************************************************************************
 **************************** Local Functions *****************************
 **************************************************************************/
static void MMWave_internalDelChirp (MMWave_Profile* ptrMMWaveProfile, MMWave_Chirp* ptrMMWaveChirp);
static void MMWave_internalDelProfile (MMWave_MCB* ptrMMWaveMCB, MMWave_Profile* ptrMMWaveProfile);

/**************************************************************************
 ******************* mmWave Configuration Functions ***********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      This is an *internal* function which is used to delete the chirp
 *      associated with a profile. The function does not hold the critical
 *      section.
 *
 *  @param[in]  ptrMMWaveProfile
 *      Pointer to the profile
 *  @param[in]  ptrMMWaveChirp
 *      Pointer to the chirp to be deleted
 *
 *  \ingroup MMWAVE_CTRL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void MMWave_internalDelChirp
(
    MMWave_Profile* ptrMMWaveProfile,
    MMWave_Chirp*   ptrMMWaveChirp
)
{
    /* Remove the chirp from the profile. */
    MMWave_listRemoveNode ((MMWave_ListNode**)&ptrMMWaveProfile->ptrChirpList, (MMWave_ListNode*)ptrMMWaveChirp);

    /* Decrement the number of chirps which are linked to the profile: */
    ptrMMWaveProfile->numChirps--;

    /* Cleanup the chirp memory: */
    HeapP_free(&gMmwHeapObj, (void *)ptrMMWaveChirp);
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the chirp from an existing profile.
 *
 *  @param[in]  profileHandle
 *      Handle to the profile to which the chirp is to be deleted
 *  @param[in]  chirpHandle
 *      Handle to the chirp to be deleted
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_delChirp
(
    MMWave_ProfileHandle    profileHandle,
    MMWave_ChirpHandle      chirpHandle,
    int32_t*                errCode
)
{
    MMWave_Chirp*       ptrMMWaveChirp;
    MMWave_Profile*     ptrMMWaveProfile;
    int32_t             retVal = MINUS_ONE;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: Validate the arguments */
    if ((profileHandle == NULL) || (chirpHandle == NULL))
    {
        /* Error: Invalid arguments */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Get the profile & chirp handle: */
    ptrMMWaveProfile = (MMWave_Profile*)profileHandle;
    ptrMMWaveChirp   = (MMWave_Chirp*)chirpHandle;

    /* Sanity Check: Each profile is linked to the mmWave module */
    DebugP_assert (ptrMMWaveProfile->ptrMMWaveMCB != NULL);

    /* Sanity Check: Are there any chirps to be deleted */
    if (ptrMMWaveProfile->numChirps == 0U)
    {
        /* Error: There are no chirps to delete. This is an invalid usage from the application */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Critical Section Enter: Protect the 'Chirp List' */
    SemaphoreP_pend (&(ptrMMWaveProfile->ptrMMWaveMCB->cfgSemHandle), SystemP_WAIT_FOREVER);

    /* Delete the chirp: */
    MMWave_internalDelChirp (ptrMMWaveProfile, ptrMMWaveChirp);

    /* Critical Section Exit: */
    SemaphoreP_post (&(ptrMMWaveProfile->ptrMMWaveMCB->cfgSemHandle));

    /* Setup the return value: */
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to add the chirp to an existing profile.
 *
 *  @param[in]  profileHandle
 *      Handle to the profile to which the chirp is to be added
 *  @param[in]  ptrChirpCfg
 *      Pointer to the chirp configuration
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the chirp
 *  @retval
 *      Error   -   NULL
 */
MMWave_ChirpHandle MMWave_addChirp
(
    MMWave_ProfileHandle    profileHandle,
    const T_RL_API_SENS_PER_CHIRP_CFG*     ptrChirpCfg,
    const T_RL_API_SENS_PER_CHIRP_CTRL*    ptrChirpCtrl,
    int32_t*                errCode
)
{
    MMWave_Chirp*           ptrMMWaveChirp;
    MMWave_Profile*         ptrMMWaveProfile;
    MMWave_ChirpHandle      retHandle = NULL;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: Validate the arguments */
    if ((profileHandle == NULL) || (ptrChirpCfg == NULL) || (ptrChirpCtrl == NULL))
    {
        /* Error: Invalid argument */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Get the profile handle: */
    ptrMMWaveProfile = (MMWave_Profile*)profileHandle;

    /* Sanity Check: Each profile is linked to the mmWave module */
    DebugP_assert (ptrMMWaveProfile->ptrMMWaveMCB != NULL);

    /* Allocate memory for the chirp: */
    ptrMMWaveChirp = (MMWave_Chirp*) HeapP_alloc(&gMmwHeapObj, (sizeof(MMWave_Chirp)));
    if (ptrMMWaveChirp == NULL)
    {
        /* Error: Out of memory */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ENOMEM, 0);
        goto exit;
    }

    /* Initialize the allocated memory for the chirp: */
    memset ((void*)ptrMMWaveChirp, 0, sizeof(MMWave_Chirp));

    /* Populate the Chirp: */
    memcpy ((void *)&ptrMMWaveChirp->chirpCfg, (const void*)ptrChirpCfg, sizeof(T_RL_API_SENS_PER_CHIRP_CFG));
    memcpy ((void *)&ptrMMWaveChirp->chirpCtrl, (const void*)ptrChirpCtrl, sizeof(T_RL_API_SENS_PER_CHIRP_CTRL));
    ptrMMWaveChirp->ptrMMWaveProfile = ptrMMWaveProfile;

    /* Critical Section Enter: Protect the 'Chirp List' */
    SemaphoreP_pend (&(ptrMMWaveProfile->ptrMMWaveMCB->cfgSemHandle), SystemP_WAIT_FOREVER);

    /* Add the chirp to the profile list */
    MMWave_listCat ((MMWave_ListNode**)&ptrMMWaveProfile->ptrChirpList, (MMWave_ListNode**)&ptrMMWaveChirp);

    /* Increment the number of chirps which are linked to the profile: */
    ptrMMWaveProfile->numChirps++;

    /* Critical Section Exit: */
    SemaphoreP_post (&(ptrMMWaveProfile->ptrMMWaveMCB->cfgSemHandle));

    /* Setup the return handle: */
    retHandle = (MMWave_ChirpHandle)ptrMMWaveChirp;

exit:
    return retHandle;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the chirp configuration given
 *      the chirp handle
 *
 *  @param[in]  chirpHandle
 *      Handle to the chirp
 *  @param[out] ptrChirpCfg
 *      Pointer to the chirp configuration populated by the API
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_getChirpCfg
(
    MMWave_ChirpHandle  chirpHandle,
    T_RL_API_SENS_PER_CHIRP_CFG*       ptrChirpCfg,
    T_RL_API_SENS_PER_CHIRP_CTRL*      ptrChirpCtrl,
    int32_t*            errCode
)
{
    MMWave_Chirp*   ptrMMWaveChirp;
    int32_t         retVal = MINUS_ONE;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: Validate the arguments */
    if ((chirpHandle == NULL) || (ptrChirpCfg == NULL) || (ptrChirpCtrl == NULL))
    {
        /* Error: Invalid arguments */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Get the pointer to the Chirp: */
    ptrMMWaveChirp = (MMWave_Chirp*)chirpHandle;

    /* Chirps are always linked to profiles and profiles to the mmWave control module. */
    DebugP_assert (ptrMMWaveChirp->ptrMMWaveProfile != NULL);
    DebugP_assert (ptrMMWaveChirp->ptrMMWaveProfile->ptrMMWaveMCB != NULL);

    /* Copy over the configuration: */
    memcpy ((void*)ptrChirpCfg, (void*)&ptrMMWaveChirp->chirpCfg, sizeof(T_RL_API_SENS_PER_CHIRP_CFG));
    memcpy ((void*)ptrChirpCtrl, (void*)&ptrMMWaveChirp->chirpCtrl, sizeof(T_RL_API_SENS_PER_CHIRP_CTRL));

    /* Setup the return value: */
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the number of chirps attached to a profile
 *
 *  @param[in]  profileHandle
 *      Handle to the profile
 *  @param[out] numChirps
 *      Number of chirps attached to a profile
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_getNumChirps
(
    MMWave_ProfileHandle    profileHandle,
    uint32_t*               numChirps,
    int32_t*                errCode
)
{
    MMWave_Profile*     ptrMMWaveProfile;
    int32_t             retVal = MINUS_ONE;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: Validate the arguments */
    if ((profileHandle == NULL) || (numChirps == NULL))
    {
        /* Error: Invalid arguments */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Get the pointer to the Profile: */
    ptrMMWaveProfile = (MMWave_Profile*)profileHandle;

    /* Sanity Check: Profiles are always linked to the control module */
    DebugP_assert (ptrMMWaveProfile->ptrMMWaveMCB != NULL);

    /* Get the number of chirps: */
    *numChirps = ptrMMWaveProfile->numChirps;

    /* Setup the return value: */
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function can be used by the application to get the chirp handle
 *      at the specified index. If the index exceeds the number of chirps
 *      configured the function will fail with the error code.
 *
 *  @param[in]  profileHandle
 *      Handle to the profile
 *  @param[in]  chirpIndex
 *      Chirp Index for which the handle is needed. Set to 1 to get the
 *      first chirp index etc
 *  @param[out] chirpHandle
 *      Populated chirp handle
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_getChirpHandle
(
    MMWave_ProfileHandle    profileHandle,
    uint32_t                chirpIndex,
    MMWave_ChirpHandle*     chirpHandle,
    int32_t*                errCode
)
{
    MMWave_Profile*     ptrMMWaveProfile;
    MMWave_Chirp*       ptrMMWaveChirp;
    uint32_t            index  = 1U;
    int32_t             retVal = MINUS_ONE;
    int32_t             endProcessing = 0;

    /* Initialize the error code: */
    *errCode     = 0;

    /* Sanity Check: Validate the arguments */
    if ((profileHandle == NULL) || (chirpHandle == NULL))
    {
        /* Error: Invalid arguments */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Get the pointer to the Profile: */
    ptrMMWaveProfile = (MMWave_Profile*)profileHandle;

    /* Sanity Check: Profiles are always linked to the control module */
    DebugP_assert (ptrMMWaveProfile->ptrMMWaveMCB != NULL);

    /* Initialize the chirp handle */
    *chirpHandle = NULL;

    /* Critical Section Enter: Protect the 'Chirp List' */
    SemaphoreP_pend (&(ptrMMWaveProfile->ptrMMWaveMCB->cfgSemHandle), SystemP_WAIT_FOREVER);

    /* Get the head of the chirp list: */
    ptrMMWaveChirp = (MMWave_Chirp*)MMWave_listGetHead ((MMWave_ListNode**)&ptrMMWaveProfile->ptrChirpList);
    while (endProcessing == 0)
    {
        /* Have we reached the end of the list? */
        if (ptrMMWaveChirp == NULL)
        {
            /* YES: Control comes here indicates that the chirp index specified exceeds the
             * configured number of chirps. We are done with the processing */
            *errCode      = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
            endProcessing = 1;
        }
        else
        {
            /* Is this what we are looking for? */
            if (index == chirpIndex)
            {
                /* YES: Setup the chirp handle. */
                *chirpHandle  = (MMWave_ChirpHandle)ptrMMWaveChirp;
                retVal        = 0;
                endProcessing = 1;
            }

            /* Get the next element: */
            index = index + 1U;
            ptrMMWaveChirp = (MMWave_Chirp*)MMWave_listGetNext ((MMWave_ListNode*)ptrMMWaveChirp);
        }
    }

    /* Critical Section Exit: */
    SemaphoreP_post (&(ptrMMWaveProfile->ptrMMWaveMCB->cfgSemHandle));

exit:
    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function is used to add the profile with the specific
 *      profile configuration.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave module
 *  @param[in]  ptrProfileCfg
 *      Pointer to the profile configuration
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the profile
 *  @retval
 *      Error   -   NULL
 */
MMWave_ProfileHandle MMWave_addProfile
(
    MMWave_Handle           mmWaveHandle,
    const T_RL_API_SENS_CHIRP_PROF_COMN_CFG*   ptrProfileCfg,
    const T_RL_API_SENS_CHIRP_PROF_TIME_CFG*   ptrProfileTimeCfg,
    int32_t*                errCode
)
{
    MMWave_MCB*             ptrMMWaveMCB;
    MMWave_Profile*         ptrMMWaveProfile;
    MMWave_ProfileHandle    retHandle = NULL;

    /* Initialize the error code: */
    *errCode = 0;

    if(!isHeapMemAllocated)
    {
        /* create heap for profile, chirp and BPM config. */
        HeapP_construct(&gMmwHeapObj, gMmwHeapMem, MMWAVE_HEAP_MEM_SIZE);
    }

    /* Sanity Check: Validate the arguments */
    if ((mmWaveHandle == NULL) || (ptrProfileCfg == NULL) || (ptrProfileTimeCfg == NULL))
    {
        /* Error: Invalid arguments */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Get the pointer to the mmWave MCB: */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;

    /* Allocate memory for the Profile: */
    ptrMMWaveProfile = (MMWave_Profile*)HeapP_alloc(&gMmwHeapObj, (sizeof(MMWave_Profile)));
    if (ptrMMWaveProfile == NULL)
    {
        /* Error: Out of memory */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ENOMEM, 0);
        goto exit;
    }

    /* Initialize the allocated memory: */
    memset ((void *)ptrMMWaveProfile, 0, sizeof(MMWave_Profile));

    /* Populate the profile: */
    memcpy ((void*)&ptrMMWaveProfile->profileCfg, (const void*)ptrProfileCfg, sizeof(T_RL_API_SENS_CHIRP_PROF_COMN_CFG));
    memcpy ((void*)&ptrMMWaveProfile->profileTimeCfg, (const void*)ptrProfileTimeCfg, sizeof(T_RL_API_SENS_CHIRP_PROF_TIME_CFG));

    ptrMMWaveProfile->ptrMMWaveMCB = ptrMMWaveMCB;

    /* Critical Section Enter: Protect the 'Profile List' */
    SemaphoreP_pend (&(ptrMMWaveMCB->cfgSemHandle), SystemP_WAIT_FOREVER);

    /* Add the profile to the Profile List  */
    MMWave_listAdd ((MMWave_ListNode**)&ptrMMWaveMCB->ptrProfileList, (MMWave_ListNode*)ptrMMWaveProfile);

    /* Critical Section Exit: */
    SemaphoreP_post (&(ptrMMWaveMCB->cfgSemHandle));

    /* Profile has been successfully registered */
    retHandle = (MMWave_ProfileHandle)ptrMMWaveProfile;

exit:
    return retHandle;
}

/**
 *  @b Description
 *  @n
 *      This is an *internal* function which is used to delete the profile
 *      The function does not hold the critical section.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the mmWave control module
 *  @param[in]  ptrMMWaveProfile
 *      Pointer to the profile to be deleted
 *
 *  \ingroup MMWAVE_CTRL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void MMWave_internalDelProfile (MMWave_MCB* ptrMMWaveMCB, MMWave_Profile* ptrMMWaveProfile)
{
    /* Remove the profile from the Profile List  */
    MMWave_listRemoveNode ((MMWave_ListNode**)&ptrMMWaveMCB->ptrProfileList, (MMWave_ListNode*)ptrMMWaveProfile);

    /* Cleanup the profile memory: */
    HeapP_free(&gMmwHeapObj, (void *)ptrMMWaveProfile);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the profile. This will also delete all
 *      the chirps which are still attached to the profile.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave module
 *  @param[in]  profileHandle
 *      Handle to the profile to be deleted
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_delProfile
(
    MMWave_Handle           mmWaveHandle,
    MMWave_ProfileHandle    profileHandle,
    int32_t*                errCode
)
{
    MMWave_MCB*             ptrMMWaveMCB;
    MMWave_Profile*         ptrMMWaveProfile;
    MMWave_Chirp*           ptrMMWaveChirp;
    int32_t                 retVal = MINUS_ONE;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: Validate the arguments */
    if ((mmWaveHandle == NULL) || (profileHandle == NULL))
    {
        /* Error: Invalid arguments */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Setup the pointers: */
    ptrMMWaveMCB     = (MMWave_MCB*)mmWaveHandle;
    ptrMMWaveProfile = (MMWave_Profile*)profileHandle;

    /* Critical Section Enter: Protect the 'Profile & Chirp List' */
    SemaphoreP_pend (&(ptrMMWaveMCB->cfgSemHandle), SystemP_WAIT_FOREVER);

    /* Cycle through all the registered chirps: */
    ptrMMWaveChirp = (MMWave_Chirp*)MMWave_listGetHead ((MMWave_ListNode**)&ptrMMWaveProfile->ptrChirpList);
    while (ptrMMWaveChirp != NULL)
    {
        /* Delete the chirp: Use the internal API since the semaphore is already held. */
        MMWave_internalDelChirp (ptrMMWaveProfile, ptrMMWaveChirp);

        /* Cycle through the list again and get the new head. */
        ptrMMWaveChirp = (MMWave_Chirp*)MMWave_listGetHead ((MMWave_ListNode**)&ptrMMWaveProfile->ptrChirpList);
    }

    /* Delete the profile: */
    MMWave_internalDelProfile (ptrMMWaveMCB, ptrMMWaveProfile);

    /* Critical Section Exit: */
    SemaphoreP_post (&(ptrMMWaveMCB->cfgSemHandle));

    /* Setup the return value: */
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the number of profiles which have been added.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave module
 *  @param[out] numProfiles
 *      Number of added profiles populated by the API
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_getNumProfiles(MMWave_Handle mmWaveHandle, uint32_t* numProfiles, int32_t* errCode)
{
    MMWave_MCB*         ptrMMWaveMCB;
    MMWave_Profile*     ptrMMWaveProfile;
    int32_t             retVal = MINUS_ONE;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: Validate the arguments */
    if ((mmWaveHandle == NULL) || (numProfiles == NULL))
    {
        /* Error: Invalid arguments */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Get the pointer to the mmWave control module: */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;

    /* Critical Section Enter: Protect the 'Profile List' */
    SemaphoreP_pend (&(ptrMMWaveMCB->cfgSemHandle), SystemP_WAIT_FOREVER);

    /* Initialize the number of profiles */
    *numProfiles = 0U;

    /* Cycle through the profile list */
    ptrMMWaveProfile = (MMWave_Profile*)MMWave_listGetHead ((MMWave_ListNode**)&ptrMMWaveMCB->ptrProfileList);
    while (ptrMMWaveProfile != NULL)
    {
        /* Increment the number of profiles */
        *numProfiles = *numProfiles + 1U;
        ptrMMWaveProfile = (MMWave_Profile*)MMWave_listGetNext ((MMWave_ListNode*)ptrMMWaveProfile);
    }

    /* Critical Section Exit: */
    SemaphoreP_post (&(ptrMMWaveMCB->cfgSemHandle));

    /* Setup the return value: */
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the profile handle for the specific profile
 *      identifier.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave module
 *  @param[in]  profileId
 *      Profile Id
 *  @param[out] profileHandle
 *      Handle to the profile populated by the API
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_getProfileHandle
(
    MMWave_Handle           mmWaveHandle,
    uint8_t                 profileId,
    MMWave_ProfileHandle*   profileHandle,
    int32_t*                errCode
)
{
    MMWave_MCB*         ptrMMWaveMCB;
    MMWave_Profile*     ptrMMWaveProfile;
    int32_t             retVal = MINUS_ONE;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: Validate the arguments */
    if ((mmWaveHandle == NULL) || (profileHandle == NULL))
    {
        /* Error: Invalid arguments */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Setup the pointers: */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;

    /* Critical Section Enter: Protect the 'Profile List' */
    SemaphoreP_pend (&(ptrMMWaveMCB->cfgSemHandle), SystemP_WAIT_FOREVER);

    /* Cycle through the profile list */
    ptrMMWaveProfile = (MMWave_Profile*)MMWave_listGetHead ((MMWave_ListNode**)&ptrMMWaveMCB->ptrProfileList);
    while (ptrMMWaveProfile != NULL)
    {
        /* Is this what we are looking for? */
        if (profileId == ptrMMWaveProfile->profileId)
        {
            /* YES: Setup the profile handle */
            *profileHandle = (MMWave_ProfileHandle)ptrMMWaveProfile;
            retVal = 0;
            break;
        }

        /* Get the next element: */
        ptrMMWaveProfile = (MMWave_Profile*)MMWave_listGetNext ((MMWave_ListNode*)ptrMMWaveProfile);
    }

    /* Critical Section Exit: */
    SemaphoreP_post (&(ptrMMWaveMCB->cfgSemHandle));

    /* Did we find a match? */
    if (ptrMMWaveProfile == NULL)
    {
        /* Error: No matching profile identifier found. Setup the error code. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ENOTFOUND, 0);
    }

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the chirp configuration given
 *      the chirp handle
 *
 *  @param[in]  profileHandle
 *      Handle to the profile
 *  @param[out] ptrProfileCfg
 *      Pointer to the profile configuration populated by the API
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_getProfileCfg
(
    MMWave_ProfileHandle    profileHandle,
    T_RL_API_SENS_CHIRP_PROF_COMN_CFG*         ptrProfileCfg,
    T_RL_API_SENS_CHIRP_PROF_TIME_CFG*         ptrProfileTimeCfg,
    int32_t*                errCode
)
{
    MMWave_Profile*     ptrMMWaveProfile;
    int32_t             retVal = MINUS_ONE;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: Validate the arguments */
    if ((profileHandle == NULL) || (ptrProfileCfg == NULL) || (ptrProfileTimeCfg == NULL))
    {
        /* Error: Invalid arguments */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Get the pointer to the Profile: */
    ptrMMWaveProfile = (MMWave_Profile*)profileHandle;

    /* Sanity Check: Profiles are always linked to the control module */
    DebugP_assert (ptrMMWaveProfile->ptrMMWaveMCB != NULL);

    /* Copy over the configuration: */
    memcpy ((void*)ptrProfileCfg, (void*)&ptrMMWaveProfile->profileCfg, sizeof(T_RL_API_SENS_CHIRP_PROF_COMN_CFG));
    memcpy ((void*)ptrProfileTimeCfg, (void*)&ptrMMWaveProfile->profileTimeCfg, sizeof(T_RL_API_SENS_CHIRP_PROF_TIME_CFG));

    /* Setup the return value: */
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to flush & clean up the configuration which is stored in
 *      the mmWave module. Due to memory constraints applications could use this API
 *      to reduce the amount of memory which is being used.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave module
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_flushCfg
(
    MMWave_Handle   mmWaveHandle,
    int32_t*        errCode
)
{
    MMWave_MCB*       ptrMMWaveMCB;
    MMWave_Profile*   ptrMMWaveProfile;
    MMWave_Chirp*     ptrMMWaveChirp;
    int32_t           retVal = MINUS_ONE;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: Validate the arguments */
    if (mmWaveHandle == NULL)
    {
        /* Error: Invalid argument */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Setup the pointers: */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;

    /* Critical Section Enter: Protect lists */
    SemaphoreP_pend (&(ptrMMWaveMCB->cfgSemHandle), SystemP_WAIT_FOREVER);

    /* Cycle through all the profiles: */
    ptrMMWaveProfile = (MMWave_Profile*)MMWave_listGetHead ((MMWave_ListNode**)&ptrMMWaveMCB->ptrProfileList);
    while (ptrMMWaveProfile != NULL)
    {
        /* Cycle through all the registered chirps for that profile */
        ptrMMWaveChirp = (MMWave_Chirp*)MMWave_listGetHead ((MMWave_ListNode**)&ptrMMWaveProfile->ptrChirpList);
        while (ptrMMWaveChirp != NULL)
        {
            /* Delete the chirp: Use the internal API since the semaphore is already held. */
            MMWave_internalDelChirp (ptrMMWaveProfile, ptrMMWaveChirp);

            /* Cycle through the list again and get the new head. */
            ptrMMWaveChirp = (MMWave_Chirp*)MMWave_listGetHead ((MMWave_ListNode**)&ptrMMWaveProfile->ptrChirpList);
        }

        /* Delete the profile: */
        MMWave_internalDelProfile (ptrMMWaveMCB, ptrMMWaveProfile);

        /* Cycle through the next profile and get the new head. */
        ptrMMWaveProfile = (MMWave_Profile*)MMWave_listGetHead ((MMWave_ListNode**)&ptrMMWaveMCB->ptrProfileList);
    }

    /* Critical Section Exit: */
    SemaphoreP_post (&(ptrMMWaveMCB->cfgSemHandle));

    /* Setup the return value: */
    retVal = 0;

exit:
    return retVal;
}

