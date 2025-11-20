let common = system.getScript("/common");

const staticConfig_m4f = [
    {
        name: "MDO0",
    },
];

function getStaticConfigArr() {


    return staticConfig_m4f;
}

function getInterfaceName(inst) {
    return "MDO";
}


let soc = {

    getStaticConfigArr,
    getInterfaceName,
};

exports = soc;