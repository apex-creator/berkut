
let common = system.getScript("/common");

/* On M4F, interrupt number as specified in TRM is input to the NVIC but from M4 point of view there are 16 internal interrupts
 * and then the NVIC input interrupts start, hence we need to add +16 to the value specified by TRM */
const staticConfig_m4f = [
    {
        name: "INA",
    },
];

function getStaticConfigArr() {


    return staticConfig_m4f;
}

function getInterfaceName(inst) {
    return "INA";
}

function isMakeInstanceRequired() {
    return false;
}


let soc = {

    getStaticConfigArr,
    getInterfaceName,
    isMakeInstanceRequired,
};

exports = soc;
