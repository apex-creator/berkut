
let common = system.getScript("/common");


let timerClockSourceConfig = [
    {
        "name": "OSC_CLK",
    },
    {
        "name": "SLOW_CLK"
    }
];

function getTimerClockSourceValue(instance) {
    let clkSelMuxValue = 0;

    switch(instance.clkSource) {
        default:
        case "OSC_CLK":
            clkSelMuxValue = 0x000;
            break;
        case "SLOW_CLK":
            clkSelMuxValue = 0x00003330;
            break;
    }
    return clkSelMuxValue;
}

let staticConfig_m4f = [
    {
        name: "M4_SYSTICK",
        timerBaseAddr: 0xE000E010, /* Setting to SYST_CSR as defined by ARMv7-M */
        timerHwiIntNum: 15,
        timerInputPreScaler: 1, /* NOT USED */
        clkSelMuxAddr: 0, /* NOT USED */
        disableClkSourceConfig: true,
    },
    {
        timerName: "APPSS_RTIA",
        timerBaseAddr: 0x56F7F000,
        timerHwiIntNum: 43,
        timerInputPreScaler: 1,
        clkSelMuxAddr: 0x56040000 + 0x34,
        disableClkSourceConfig: false,
        lockUnlockDomain: "SOC_DOMAIN_ID_APP_RCM",
        lockUnlockPartition: 0,
    },
];

function getStaticConfigArr() {

    return staticConfig_m4f;
}

function getTimerClockSourceHz(clkSource) {
    let clkSourceHz = 0;
    switch(clkSource) {
        default:
        case "OSC_CLK":
            clkSourceHz = 40*1000000;
        break;
        case "SLOW_CLK":
            clkSourceHz = 32*1000;
        break;
    }
    return clkSourceHz;
}

function getInterfaceName(inst) {

    return "APPSS_RTI";
}

function getTimerClockSourceConfigArr() {

    return timerClockSourceConfig;
}

function getDefaultTimerClockSourceMhz() {

    return 40*1000000;
}

function getBlockedTimers() {

    return ['M4_SYSTICK'];
}

exports = {
    getStaticConfigArr,
    getTimerClockSourceConfigArr,
    getTimerClockSourceValue,
    getTimerClockSourceHz,
    getInterfaceName,
    getBlockedTimers,
    oneShotModeSupport: false,
};

