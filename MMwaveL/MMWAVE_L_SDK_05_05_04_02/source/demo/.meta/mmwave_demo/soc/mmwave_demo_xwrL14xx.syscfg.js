
let common = system.getScript("/common");

const staticConfig_m4f = [
    {
        name: "MMWAVE_DEMO",
    },
];

function getStaticConfigArr() {


    return staticConfig_m4f;
}

function getInterfaceName(inst) {
    return "MMWAVE_DEMO";
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
