let common = system.getScript("/common");

const hwa_config_m4fss = [
    {
        name                        : "HWA0",
        ctrlBaseAddr                : "CSL_APP_HWA_CFG_U_BASE",
        paramBaseAddr               : "CSL_APP_HWA_PARAM_U_BASE",
        ramBaseAddr                 : "CSL_APP_HWA_WINDOW_RAM_U_BASE",
        numHwaParamSets             : "SOC_HWA_NUM_PARAM_SETS",
        intNumParamSet              : "(16U + CSL_APPSS_INTR_HWASS_PARAMDONE_INT)",
        intNumDone                  : "(16U + CSL_APPSS_INTR_HWASS_LOOP_INT)",
        numDmaChannels              : "SOC_HWA_NUM_DMA_CHANNEL",
        accelMemBaseAddr            : "CSL_APP_HWA_DMA0_U_BASE",
        accelMemSize                : "SOC_HWA_MEM_SIZE",
        isConcurrentAccessAllowed   : "true",
        isCompressionEnginePresent  : "true",
        clockIds                    : [ "SOC_RcmPeripheralId_HWASS" ],
    },
];

function getConfigArr() {


    return hwa_config_m4fss;
}

exports = {
    getConfigArr,
};
