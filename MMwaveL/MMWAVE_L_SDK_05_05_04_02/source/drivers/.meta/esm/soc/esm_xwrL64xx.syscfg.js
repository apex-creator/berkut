let common = system.getScript("/common");

const esm_config_m4fss = [
    {
        name                : "APP_ESM",
        regBaseAddr         : "CSL_APP_ESM_U_BASE",
        ctrlBaseAddr        : "CSL_APP_CTRL_U_BASE",
        numGroupErr         : "ESM_NUM_INTR_PER_GROUP",              
        highPrioIntNum      : "2",
        lowPrioIntNum       : "16 + 1",
        intHighPriority     : 0x7,
        intLowPriority      : 4,
        clockIds            : [ "SOC_RcmPeripheralId_APPSS_ESM" ],
    },
];

function getConfigArr() {

    return esm_config_m4fss;
}

function getMaxNotifier() {
   /* Max number of notifer ESM instance can support is 4 */
    return 4;
}

exports = {
    getConfigArr,
    getMaxNotifier,
};