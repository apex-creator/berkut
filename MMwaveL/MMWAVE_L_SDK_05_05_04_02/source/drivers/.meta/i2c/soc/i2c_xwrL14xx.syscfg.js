
let common = system.getScript("/common");

let i2c_func_clk = 40000000;

/* On M4F, interrupt number as specified in TRM is input to the NVIC but from M4 point of view there are 16 internal interrupts
 * and then the NVIC input interrupts start, hence we need to add +16 to the value specified by TRM */
const staticConfig_m4f = [
    {
        name: "I2C0",
        baseAddr: "CSL_APP_I2C_U_BASE",
        intNum: 16 + 19,
        eventId: 0,
        funcClk: i2c_func_clk,
        clockIds: [ "SOC_RcmPeripheralId_APPSS_I2C" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_APPSS_I2C",
                clkId   : "SOC_RcmPeripheralClockSource_OSC_CLK",
                clkRate : i2c_func_clk,
            },
        ],
    },
];

function getStaticConfigArr() {


    return staticConfig_m4f;
}

function getInterfaceName(inst) {
    return "I2C";
}

function isMakeInstanceRequired() {
    return false;
}

function isFrequencyDefined()
{
    return true;
}

let soc = {

    getStaticConfigArr,
    getInterfaceName,
    isMakeInstanceRequired,
    isFrequencyDefined,
};

exports = soc;
