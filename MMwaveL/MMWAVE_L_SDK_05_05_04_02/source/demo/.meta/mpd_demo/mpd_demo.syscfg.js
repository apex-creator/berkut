
let common = system.getScript("/common");
let soc = system.getScript(`/demo/soc/demo_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("mpd_demo");

    return system.getScript(`/demo/mpd_demo/${driverVer}/mpd_demo_${driverVer}`);
}

exports = getModule();
