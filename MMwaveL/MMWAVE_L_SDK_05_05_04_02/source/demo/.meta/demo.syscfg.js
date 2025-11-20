
let common = system.getScript("/common");
let soc = system.getScript(`/demo/soc/demo_${common.getSocName()}`);

exports = {
    displayName: "TI Demo",
    templates: [
        {
            name: "/demo/demo/cli_mpd_demo_config.h.xdt",
            outputPath: "ti_cli_mpd_demo_config.h",
            alwaysRun: true,
        },
        {
            name: "/demo/demo/cli_mmwave_demo_config.h.xdt",
            outputPath: "ti_cli_mmwave_demo_config.h",
            alwaysRun: true,
        },
    ],
    topModules: soc.getTopModules(),
};
