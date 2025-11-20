
let common = system.getScript("/common");

const driverVer = {
    "mpd_demo": {
        version: "v0",
    },
    "mmwave_demo": {
        version: "v0",
    }
};
const topModules = [
    "/demo/mpd_demo/mpd_demo",
    "/demo/mmwave_demo/mmwave_demo",


];

exports = {
    getTopModules: function() {
        return topModules;
    },
    getDriverVer: function(driverName) {
        return driverVer[driverName].version;
    },
};
