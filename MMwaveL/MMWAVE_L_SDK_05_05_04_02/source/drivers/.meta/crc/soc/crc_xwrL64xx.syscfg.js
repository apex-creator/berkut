
let common = system.getScript("/common");

/* On M4F, interrupt number as specified in TRM is input to the NVIC but from M4 point of view there are 16 internal interrupts
 * and then the NVIC input interrupts start, hence we need to add +16 to the value specified by TRM */
const crc_config_m4f = [
    {
        name: "CRC",
        baseAddr: "CSL_APP_CRC_U_BASE",
        intrNum: 16 + 20,
        clockIds: [ "SOC_RcmPeripheralId_APPSS_CRC" ],
    },
];

function getConfigArr() {

    return crc_config_m4f;
}

exports = {
    getConfigArr,
};
