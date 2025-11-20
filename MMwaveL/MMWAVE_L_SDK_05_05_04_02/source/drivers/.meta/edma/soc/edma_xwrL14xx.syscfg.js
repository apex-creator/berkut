let common = system.getScript("/common");

const edma_config_m4fss = [
    {
        name: "EDMA_APPSS_A",
        baseAddr: "CSL_APP_TPCC_A_U_BASE",
        compIntrNumber: "(16U + CSL_APPSS_INTR_APPSS_TPCC1_INTAGG)",
        intrAggEnableAddr: "CSL_APP_CTRL_U_BASE + CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_MASK",
        intrAggStatusAddr: "CSL_APP_CTRL_U_BASE + CSL_APP_CTRL_APPSS_TPCC_A_INTAGG_STATUS",
        maxDmaChannels: "64",
        maxTcc: "64",
        maxPaRAM: "128",
        maxRegions: "8",
        maxQueue: "2",
        clockIds: [ "SOC_RcmPeripheralId_APPSS_EDMA" ],
        supportReservedChannelConfig: true,
        defaultOwnDmaChannelStart_m4fss0_0: "0",
        defaultOwnDmaChannelEnd_m4fss0_0: "63",
        defaultOwnQdmaChannelStart_m4fss0_0: "0",
        defaultOwnQdmaChannelEnd_m4fss0_0: "7",
        defaultOwnTccStart_m4fss0_0: "0",
        defaultOwnTccEnd_m4fss0_0: "63",
        defaultOwnParamStart_m4fss0_0: "0",
        defaultOwnParamEnd_m4fss0_0: "127",
        defaultReservedDmaChannelStart_m4fss0_0: "0",
        defaultReservedDmaChannelEnd_m4fss0_0: "0",
    },
    {
        name: "EDMA_APPSS_B",
        baseAddr: "CSL_APP_TPCC_B_U_BASE",
        compIntrNumber: "(16U + CSL_APPSS_INTR_APPSS_TPCC2_INTAGG)",
        intrAggEnableAddr: "CSL_APP_CTRL_U_BASE + CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_MASK",
        intrAggStatusAddr: "CSL_APP_CTRL_U_BASE + CSL_APP_CTRL_APPSS_TPCC_B_INTAGG_STATUS",
        maxDmaChannels: "64",
        maxTcc: "64",
        maxPaRAM: "128",
        maxRegions: "8",
        maxQueue: "2",
        clockIds: [ "SOC_RcmPeripheralId_APPSS_EDMA" ],
        supportReservedChannelConfig: true,
        defaultOwnDmaChannelStart_m4fss0_0: "0",
        defaultOwnDmaChannelEnd_m4fss0_0: "63",
        defaultOwnQdmaChannelStart_m4fss0_0: "0",
        defaultOwnQdmaChannelEnd_m4fss0_0: "7",
        defaultOwnTccStart_m4fss0_0: "0",
        defaultOwnTccEnd_m4fss0_0: "63",
        defaultOwnParamStart_m4fss0_0: "0",
        defaultOwnParamEnd_m4fss0_0: "127",
        defaultReservedDmaChannelStart_m4fss0_0: "0",
        defaultReservedDmaChannelEnd_m4fss0_0: "0",
    }
];

function getConfigArr() {

    return edma_config_m4fss;
}

function getDefaultRegion() {
    let defRegion = 0;

    return defRegion;
}

function isReservedChannelSupported() {
    return true;
}

function isChannelTriggerXbarSupported() {
    return false;
}

exports = {
    getConfigArr,
    getDefaultRegion,
    isReservedChannelSupported,
    isChannelTriggerXbarSupported,
};
