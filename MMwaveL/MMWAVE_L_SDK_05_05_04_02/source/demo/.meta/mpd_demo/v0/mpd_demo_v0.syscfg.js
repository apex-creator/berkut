
let common = system.getScript("/common");
let soc = system.getScript(`/demo/mpd_demo/soc/mpd_demo_${common.getSocName()}`);
let nodeCmd = "node";
let res = null;
let mpdarray = [];
let mpdobj = [{ name: "0"},];
let isFileCopy = false;

if(system.getOS() == "win")
{
    nodeCmd = "node.exe";
}

function getStaticConfigArr() {
    return system.getScript(`/demo/mpd_demo/soc/mpd_demo_${common.getSocName()}`).getStaticConfigArr();
}

function getInstanceConfig(moduleInstance) {
    return {
        ...moduleInstance,
    };
};


function getInterfaceName(inst) {

    return "MPD_DEMO";
}

function getConfigurables()
{
    let config = [];
    let temp_mpd = [];

    config.push(

        {
            name: "CLI",
            displayName: "CLI Removal",
            default: "0",
            description: "Enable/Disable CLI Removal for a memory optimized demo",
            longDescription: `
    The CLI Removal option, if enabled, optimizes the demo application for memory by disabling CLI utility and certain features 
    dependent on CLI such as Quick Eval, ADC data logging and injection, Monitors. 
    This also helps in power saving by reducing the overall retention RAM and thereby, deep sleep power
    `,
            options: [
                {
                    name: "1",
                    displayName: "Enable"
                },
                {
                    name: "0",
                    displayName: "Disable"
                },
            ],
            onChange: function (inst, ui) {
                if(inst.CLI == "0") {
                    ui.DEVICE.hidden = true;
                    ui.userCliCfgFromFile.hidden = true;
                    ui.MPD_ENABLE.hidden = true;
                    ui.TRACKER_CLASSIFIER_ENABLE.hidden = true;
                    ui.QUICK_START.hidden = false;
                    ui.SPI_ADC_DATA_STREAMING.hidden = false;
                    ui.DYNAMIC_RECONFIG.hidden = false;
                    ui.ENABLE_MONITORS.hidden = false;
                    isFileCopy = false;
                }
                else {
                    ui.DEVICE.hidden = false;
                    ui.userCliCfgFromFile.hidden = false;
                    ui.MPD_ENABLE.hidden = false;
                    ui.TRACKER_CLASSIFIER_ENABLE.hidden = true; //set to false when enabling tracker in CLI rem flow
                    ui.QUICK_START.hidden = true;
                    ui.SPI_ADC_DATA_STREAMING.hidden = true;
                    ui.DYNAMIC_RECONFIG.hidden = true;
                    ui.ENABLE_MONITORS.hidden = true;
                    isFileCopy = true;
                }
            },
        },
        {
            name: "DEVICE",
            displayName: "DEVICE SELECT",
            default: "0",
            description: "XWRL64XX/XWRL14XX",
            hidden:true,
            options: [
                {
                    name: "1",
                    displayName: "XWRL14XX"
                },
                {
                    name: "0",
                    displayName: "XWRL64XX"
                },
            ],
        },
        {
            name: "MPD_ENABLE",
            displayName: "MPD DPU Enable",
            default: "0",
            hidden:true,
            description: "Enable/Disable MPD proc DPU in demo",
            longDescription: `
    The mpdproc DPU takes in the detected point cloud, performs db scan clustering and thresholding, to indicate whether motion or presence detected in the configured RoI.
    This is disabled by default in the demo, when CLI removal is enabled.
    
    **Note** - this cannot be enabled in tandem with tracker classifier processing. Alternatively, DYNAMIC RECONFIG option can be used for dynamic switching between the two based on the detected scene.
    
    **Prerequisites** - Please ensure required CLI configurations are added to the json file and Load from JSON is updated, when this is enabled.
    Refer to the SDK user guide or Tuning guide for configuration details.
            `,
            options: [
                {
                    name: "1",
                    displayName: "Enable"
                },
                {
                    name: "0",
                    displayName: "Disable"
                },
            ],
            onChange: function (inst, ui) {
                if(inst.MPD_ENABLE == "1") {
                    ui.TRACKER_CLASSIFIER_ENABLE.hidden = true;
                }
                else {
                    ui.TRACKER_CLASSIFIER_ENABLE.hidden = true; //set to false when enabling tracker in CLI rem flow
                }
            },
        },
        {
            name: "TRACKER_CLASSIFIER_ENABLE",
            displayName: "TRACKER CLASSIFIER DPU ENABLE",
            default: "0",
            hidden:true,
            description: "Enable/Disable DPUs for Tracker and Classification in demo",
            longDescription: `
    The trackerproc and udopproc DPUs are responsible for tracking and classifying any human vs non-human movement in the Radar's FoV.
    This is disabled by default in the demo, when CLI removal is enabled.
    
    **Note** - this cannot be enabled in tandem with mpd proc. Alternatively, DYNAMIC RECONFIG option can be used for dynamic switching between the two based on the detected scene.
    
    **Prerequisites** - Please ensure required CLI configurations are added to the json file and Load from JSON is updated, when this is enabled.
    Refer to the SDK user guide or Tuning guide for configuration details.
            `,
            options: [
                {
                    name: "1",
                    displayName: "Enable"
                },
                {
                    name: "0",
                    displayName: "Disable"
                },
            ],
            onChange: function (inst, ui) {
                if(inst.TRACKER_CLASSIFIER_ENABLE == "1") {
                    ui.MPD_ENABLE.hidden = true;
                }
                else {
                    ui.MPD_ENABLE.hidden = false;
                }
            },
        },
        {
            name: "QUICK_START",
            displayName: "QUICK EVAL",
            default: "0",
            description: "Enable/Disable default led blinking config",
            longDescription: `
            Preflashed default application toggles the user LED based on presence detected within 1x1 sq mtr box infront of the radar.
            This helps in quick evaluation upon power on, if enabled.
            `,
            options: [
                {
                    name: "1",
                    displayName: "Enable"
                },
                {
                    name: "0",
                    displayName: "Disable"
                },
            ],
        },
        {
            name: "SPI_ADC_DATA_STREAMING",
            displayName: "ADC STREAMING via SPI",
            default: "0",
            description: "Enable/Disable ADC STREAMING via SPI",
            longDescription:`
    The Demo supports streaming of raw ADC data over SPI interface
    every frame during the frame idle time, if enabled.
            `,
            options: [
                {
                    name: "1",
                    displayName: "Enable"
                },
                {
                    name: "0",
                    displayName: "Disable"
                },
            ],
        },
        {
            name: "DYNAMIC_RECONFIG",
            displayName: "DYNAMIC RECONFIG",
            default: "0",
            description: "Enable/Disable profile switching between presence and tracker configuration",
            longDescription: `
            This feature enables seamless transitions between high-performance tracker processing and low-power presence detection through the designed state machine.
            It additionally achieves power saving by intelligent configuration switching, based on the detected scene, if enabled.
            `,
            options: [
                {
                    name: "1",
                    displayName: "Enable"
                },
                {
                    name: "0",
                    displayName: "Disable"
                },
            ],
        },
        {
            name: "ENABLE_MONITORS",
            displayName: "MONITORS",
            default: "0",
            description: "Enable/Disable Monitors",
            longDescription: `
    mmWaveLink Monitor Module is responsible for providing APIs for FECSS analog monitor. If enabled, it supports the following monitors: 
    - APLL & SYNTH Monitors
    - RX and TX loopback Monitors
    - TX Power Monitor
    - TX Ballbreak Monitor
    - RX Saturation Live monitor
    - Synth Frequency Live Monitor
    - Internal Signal DCBIST Monitors
    - Rx Saturation Live Monitors and Synth Frequency Monitors
    `,
            options: [
                {
                    name: "1",
                    displayName: "Enable"
                },
                {
                    name: "0",
                    displayName: "Disable"
                },
            ],
        },
        {
            name: "channelCfg_rxChCtrlBitMask",
            default: "7",
            hidden: true,
        },
        {
            name: "channelCfg_txChCtrlBitMask",
            default: "3",
            hidden: true,
        },
        {
            name: "channelCfg_miscCtrl",
            default: "0",
            hidden: true,
        },
        {
            name: "chirpComnCfg_digOutputSampRate",
            default: "18",
            hidden: true,
        },
        {
            name: "chirpComnCfg_digOutputBitsSel",
            default: "0",
            hidden: true,
        },
        {
            name: "chirpComnCfg_dfeFirSel",
            default: "0",
            hidden: true,
        },
        {
            name: "chirpComnCfg_numOfAdcSamples",
            default: "128",
            hidden: true,
        },
        {
            name: "chirpComnCfg_chirpTxMimoPatSel",
            default: "4",
            hidden: true,
        },
        {
            name: "chirpComnCfg_chirpRampEndTime",
            default: "30",
            hidden: true,
        },
        {
            name: "chirpComnCfg_chirpRxHpfSel",
            default: "0",
            hidden: true,
        },
        {
            name: "chirpTimingCfg_chirpIdleTime",
            default: "6",
            hidden: true,
        },
        {
            name: "chirpTimingCfg_chirpAdcSkipSamples",
            default: "28",
            hidden: true,
        },
        {
            name: "chirpTimingCfg_chirpTxStartTime",
            default: "0",
            hidden: true,
        },
        {
            name: "chirpTimingCfg_chirpRfFreqSlope",
            default: "90",
            hidden: true,
        },
        {
            name: "chirpTimingCfg_chirpRfFreqStart_64xx",
            default: "59.75",
            hidden: true,
        },
        {
            name: "chirpTimingCfg_chirpRfFreqStart_14xx",
            default: "77.0",
            hidden: true,
        },
        {
            name: "frameCfg_numOfChirpsInBurst",
            default: "8",
            hidden: true,
        },
        {
            name: "frameCfg_numOfChirpsAccum",
            default: "0",
            hidden: true,
        },
        {
            name: "frameCfg_burstPeriodicity",
            default: "403",
            hidden: true,
        },
        {
            name: "frameCfg_numOfBurstsInFrame",
            default: "1",
            hidden: true,
        },
        {
            name: "frameCfg_framePeriodicity",
            default: "250.0",
            hidden: true,
        },
        {
            name: "frameCfg_numOfFrames",
            default: "0",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt0_row",
            default: "0",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt0_col",
            default: "0",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt1_row",
            default: "1",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt1_col",
            default: "1",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt2_row",
            default: "0",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt2_col",
            default: "2",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt3_row",
            default: "0",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt3_col",
            default: "1",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt4_row",
            default: "1",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt4_col",
            default: "2",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt5_row",
            default: "0",
            hidden: true,
        },
        {
            name: "antGeometryCfg_vAnt5_col",
            default: "3",
            hidden: true,
        },
        {
            name: "antGeometryCfg_antDistX",
            default: "2.418",
            hidden: true,
        },
        {
            name: "antGeometryCfg_antDistZ",
            default: "2.418",
            hidden: true,
        },
        {
            name: "guiMonitor_pointCloud",
            default: "0",
            hidden: true,
        },
        {
            name: "guiMonitor_rangeProfile",
            default: "0",
            hidden: true,
        },
        {
            name: "guiMonitor_noiseProfile",
            default: "0",
            hidden: true,
        },
        {
            name: "guiMonitor_rangeAzimuthHeatMap",
            default: "0",
            hidden: true,
        },
        {
            name: "guiMonitor_rangeDopplerHeatMap",
            default: "0",
            hidden: true,
        },
        {
            name: "guiMonitor_statsInfo",
            default: "1",
            hidden: true,
        },
        {
            name: "guiMonitor_presenceInfo",
            default: "0",
            hidden: true,
        },
        {
            name: "guiMonitor_adcSamples",
            default: "0",
            hidden: true,
        },
        {
            name: "guiMonitor_trackerInfo",
            default: "0",
            hidden: true,
        },
        {
            name: "guiMonitor_microDopplerInfo",
            default: "0",
            hidden: true,
        },
        {
            name: "guiMonitor_classifierInfo",
            default: "0",
            hidden: true,
        },
        {
            name: "guiMonitor_quickEvalInfo",
            default: "0",
            hidden: true,
        },
        {
            name: "guiMonitor_pointCloudAntennaSymbols",
            default: "0",
            hidden: true,
        },
        {
            name: "steerVecCorr_enableAntSymbGen",
            default: "0",
            hidden: true,
        },
        {
            name: "steerVecCorr_enableSteeringVectorCorrection",
            default: "0",
            hidden: true,
        },
        {
            name: "steerVecCorr_enableAngleInterpolation",
            default: "0",
            hidden: true,
        },
        {
            name: "rangeCompCfg_enable",
            default: "0",
            hidden: true,
        },
        {
            name: "rangeCompCfg_detectionRangeIdx",
            default: "-1",
            hidden: true,
        },
        {
            name: "rangeCompCfg_detectionSNR",
            default: "-1",
            hidden: true,
        },
        {
            name: "rangeCompCfg_minCompRange",
            default: "-1",
            hidden: true,
        },
        {
            name: "rangeCompCfg_maxCompRange",
            default: "-1",
            hidden: true,
        },
        {
            name: "rangeCompCfg_minCompAngle1",
            default: "-90",
            hidden: true,
        },
        {
            name: "rangeCompCfg_maxCompAngle1",
            default: "90",
            hidden: true,
        },
        {
            name: "rangeCompCfg_minCompAngle2",
            default: "-90",
            hidden: true,
        },
        {
            name: "rangeCompCfg_maxCompAngle2",
            default: "90",
            hidden: true,
        },
        {
            name: "rangeCompCfg_snrDropfromAngle1ToAngle2",
            default: "0",
            hidden: true,
        },
        {
            name: "sigProcChainCfg_azimuthFftSize",
            default: "64",
            hidden: true,
        },
        {
            name: "sigProcChainCfg_elevationFftSize",
            default: "8",
            hidden: true,
        },
        {
            name: "sigProcChainCfg_motDetMode",
            default: "2",
            hidden: true,
        },
        {
            name: "sigProcChainCfg_coherentDoppler",
            default: "0",
            hidden: true,
        },
        {
            name: "sigProcChainCfg_numFrmPerMinorMotProc",
            default: "4",
            hidden: true,
        },
        {
            name: "sigProcChainCfg_numMinorMotionChirpsPerFrame",
            default: "4",
            hidden: true,
        },
        {
            name: "sigProcChainCfg_forceMinorMotionVelocityToZero",
            default: "0",
            hidden: true,
        },
        {
            name: "sigProcChainCfg_minorMotionVelocityInclusionThr",
            default: "0.5",
            hidden: true,
        },
        {
            name: "sigProcChainCfg_dopElevDimReductOrder",
            default: "0",
            hidden: true,
        },
        {
            name: "cfarCfg_averageMode",
            default: "2",
            hidden: true,
        },
        {
            name: "cfarCfg_winLen",
            default: "4",
            hidden: true,
        },
        {
            name: "cfarCfg_guardLen",
            default: "3",
            hidden: true,
        },
        {
            name: "cfarCfg_noiseDiv",
            default: "2",
            hidden: true,
        },
        {
            name: "cfarCfg_cyclicMode",
            default: "0",
            hidden: true,
        },
        {
            name: "cfarCfg_thresholdScale",
            default: "12.0",
            hidden: true,
        },
        {
            name: "cfarCfg_peakGroupingEn",
            default: "0",
            hidden: true,
        },
        {
            name: "cfarCfg_sideLobeThreshold",
            default: "0.5",
            hidden: true,
        },
        {
            name: "cfarCfg_enableLocalMaxRange",
            default: "0",
            hidden: true,
        },
        {
            name: "cfarCfg_enableLocalMaxAzimuth",
            default: "1",
            hidden: true,
        },
        {
            name: "cfarCfg_interpolateRange",
            default: "1",
            hidden: true,
        },
        {
            name: "cfarCfg_interpolateAzimuth",
            default: "1",
            hidden: true,
        },
        {
            name: "cfarCfg_lookUpTableCorrectAzimuthDom",
            default: "0",
            hidden: true,
        },
        {
            name: "aoaFovCfg_minAzimuthDeg",
            default: "-60",
            hidden: true,
        },
        {
            name: "aoaFovCfg_maxAzimuthDeg",
            default: "60",
            hidden: true,
        },
        {
            name: "aoaFovCfg_minElevationDeg",
            default: "-40",
            hidden: true,
        },
        {
            name: "aoaFovCfg_maxElevationDeg",
            default: "40",
            hidden: true,
        },
        {
            name: "rangeSelCfg_minMeters",
            default: "0.1",
            hidden: true,
        },
        {
            name: "rangeSelCfg_maxMeters",
            default: "4.0",
            hidden: true,
        },
        {
            name: "clutterRemoval_enabled",
            default: "1",
            hidden: true,
        },
        {
            name: "compRangeBiasAndRxChanPhase_rangeBias",
            default: "0.0",
            hidden: true,
        },
        {
            name: "lowPowerCfg_enable",
            default: "1",
            hidden: true,
        },
        {
            name: "factoryCalibCfg_saveEnable",
            default: "1",
            hidden: true,
        },
        {
            name: "factoryCalibCfg_restoreEnable",
            default: "0",
            hidden: true,
        },
        {
            name: "factoryCalibCfg_rxGain",
            default: "40",
            hidden: true,
        },
        {
            name: "factoryCalibCfg_txBackoff",
            default: "0",
            hidden: true,
        },
        {
            name: "factoryCalibCfg_flashOffset",
            default: "0x1FF000",
            hidden: true,
        },
        {
            name: "mpd_boundary_box_num",
            default: "0",
            hidden: true,
        },
        {
            name: "mpd_boundary_box1",
            default: "0",
            hidden: true,
        },
        {
            name: "mpd_boundary_box2",
            default: "0",
            hidden: true,
        },
        {
            name: "mpd_boundary_box3",
            default: "0",
            hidden: true,
        },
        {
            name: "mpd_boundary_box4",
            default: "0",
            hidden: true,
        },
        {
            name: "mpd_boundary_box5",
            default: "0",
            hidden: true,
        },
        {
            name: "mpd_boundary_box6",
            default: "0",
            hidden: true,
        },
        {
            name: "mpd_boundary_box7",
            default: "0",
            hidden: true,
        },
        {
            name: "mpd_boundary_box8",
            default: "0",
            hidden: true,
        },
        {
            name: "mpd_boundary_box9",
            default: "0",
            hidden: true,
        },
        {
            name: "mpd_boundary_box10",
            default: "0",
            hidden: true,
        },
        {
            name: "mpd_boundary_box11",
            default: "0",
            hidden: true,
        },
        {
            name: "mpd_boundary_box12",
            default: "0",
            hidden: true,
        },
        {
            name: "sensorPosition_x",
            default: "0",
            hidden: true,
        },
        {
            name: "sensorPosition_y",
            default: "0",
            hidden: true,
        },
        {
            name: "sensorPosition_z",
            default: "0",
            hidden: true,
        },
        {
            name: "sensorPosition_az",
            default: "0",
            hidden: true,
        },
        {
            name: "sensorPosition_el",
            default: "0",
            hidden: true,
        },
        {
            name: "minorStateCfg_en",
            default: "0",
            hidden: true,
        },
        {
            name: "minorStateCfg_point_thre1",
            default: "5",
            hidden: true,
        },
        {
            name: "minorStateCfg_point_thre2",
            default: "4",
            hidden: true,
        },
        {
            name: "minorStateCfg_snr_thre2",
            default: "40",
            hidden: true,
        },
        {
            name: "minorStateCfg_pointhist_thre1",
            default: "8",
            hidden: true,
        },
        {
            name: "minorStateCfg_pointhist_thre2",
            default: "4",
            hidden: true,
        },
        {
            name: "minorStateCfg_snrhist_thre2",
            default: "30",
            hidden: true,
        },
        {
            name: "minorStateCfg_histbuff_size",
            default: "8",
            hidden: true,
        },
        {
            name: "minorStateCfg_exit_thre",
            default: "8",
            hidden: true,
        },
        {
            name: "majorStateCfg_en",
            default: "0",
            hidden: true,
        },
        {
            name: "majorStateCfg_point_thre1",
            default: "0",
            hidden: true,
        },
        {
            name: "majorStateCfg_point_thre2",
            default: "0",
            hidden: true,
        },
        {
            name: "majorStateCfg_snr_thre2",
            default: "0",
            hidden: true,
        },
        {
            name: "majorStateCfg_pointhist_thre1",
            default: "0",
            hidden: true,
        },
        {
            name: "majorStateCfg_pointhist_thre2",
            default: "0",
            hidden: true,
        },
        {
            name: "majorStateCfg_snrhist_thre2",
            default: "0",
            hidden: true,
        },
        {
            name: "majorStateCfg_histbuff_size",
            default: "0",
            hidden: true,
        },
        {
            name: "majorStateCfg_exit_thre",
            default: "0",
            hidden: true,
        },
        {
            name: "clusterCfg_enable",
            default: "1",
            hidden: true,
        },
        {
            name: "clusterCfg_radius",
            default: "0.5",
            hidden: true,
        },
        {
            name: "clusterCfg_minpoints",
            default: "2",
            hidden: true,
        },
        {
            name: "sensorStart_frameTrigMode",
            default: "0",
            hidden: true,
        },
        {
            name: "sensorStart_chirpStartSigLbEn",
            default: "0",
            hidden: true,
        },
        {
            name: "sensorStart_frameLivMonEn",
            default: "0",
            hidden: true,
        },
        {
            name: "sensorStart_frameTrigTimerVal",
            default: "0",
            hidden: true,
        },
        {
            name: "userCliCfgFromFile",
            displayName: "Load CLI Config From JSON",
            buttonText: "Load From JSON",
            fileFilter: ".json",
            pickDirectory: false,
            nonSerializable: true,
            hidden:true,
            onLaunch: (inst) => {
                let products=system.getProducts()
                let sdkPath = ""
                let copyScriptPath = ""
                if(system.getOS() == "win") {
                    sdkPath = products[0].path.split("\\.metadata\\product.json")[0];
                    copyScriptPath = sdkPath + "//source//demo//.meta//mpd_demo//copyutil.js";
                } else {
                    sdkPath = products[0].path.split("/.metadata/product.json")[0];
                    copyScriptPath = sdkPath + "/source/demo/.meta/mpd_demo/copyutil.js";
                }
                return {
                    command: nodeCmd,
                    args: [copyScriptPath, "$browsedFile", "$comFile"],
                    initialData: "initialData",
                    inSystemPath: true,
                };
            },
            onComplete: (inst, _ui, result) => {
                if(result.data === "error") {
                    inst.fname = "ERROR LOADING CONFIG";
                    return;
                } else if(result.data === "initialData") {
                    inst.fname = "ERROR RUNNING SCRIPT";
                } else {


                    try {
                        res = JSON.parse(result.data);
                        fillConfigs(inst,res);
                    } catch (e) {
                        inst.fname = "ERROR PARSING CONFIG";
                        return;
                    }
                    /* Fill up the configurables with data from JSON file */
                    return;
                }
            },
        },       
    )
    return config;

}

function fillConfigs(inst,res)
{
    inst.channelCfg_rxChCtrlBitMask = res.channelCfg.rxChCtrlBitMask;
    inst.channelCfg_txChCtrlBitMask = res.channelCfg.txChCtrlBitMask;
    inst.channelCfg_miscCtrl = res.channelCfg.miscCtrl;

    inst.chirpComnCfg_digOutputSampRate = res.chirpComnCfg.digOutputSampRate;
    inst.chirpComnCfg_digOutputBitsSel = res.chirpComnCfg.digOutputBitsSel;
    inst.chirpComnCfg_dfeFirSel = res.chirpComnCfg.dfeFirSel;
    inst.chirpComnCfg_numOfAdcSamples = res.chirpComnCfg.numOfAdcSamples;
    inst.chirpComnCfg_chirpTxMimoPatSel = res.chirpComnCfg.chirpTxMimoPatSel;
    inst.chirpComnCfg_chirpRampEndTime = res.chirpComnCfg.chirpRampEndTime;
    inst.chirpComnCfg_chirpRxHpfSel = res.chirpComnCfg.chirpRxHpfSel;

    inst.chirpTimingCfg_chirpIdleTime = res.chirpTimingCfg.chirpIdleTime;
    inst.chirpTimingCfg_chirpAdcSkipSamples = res.chirpTimingCfg.chirpAdcSkipSamples;
    inst.chirpTimingCfg_chirpTxStartTime = res.chirpTimingCfg.chirpTxStartTime;
    inst.chirpTimingCfg_chirpRfFreqSlope = res.chirpTimingCfg.chirpRfFreqSlope;
    if(inst.DEVICE == "0")
    {
        inst.chirpTimingCfg_chirpRfFreqStart_64xx = res.chirpTimingCfg.chirpRfFreqStart;
    }
    else
    {
        inst.chirpTimingCfg_chirpRfFreqStart_14xx = res.chirpTimingCfg.chirpRfFreqStart;
    }
    inst.frameCfg_numOfChirpsInBurst = res.frameCfg.numOfChirpsInBurst;
    inst.frameCfg_numOfChirpsAccum = res.frameCfg.numOfChirpsAccum;
    inst.frameCfg_burstPeriodicity = res.frameCfg.burstPeriodicity;
    inst.frameCfg_numOfBurstsInFrame = res.frameCfg.numOfBurstsInFrame;
    inst.frameCfg_framePeriodicity = res.frameCfg.framePeriodicity;
    inst.frameCfg_numOfFrames = res.frameCfg.numOfFrames;

    inst.antGeometryCfg_vAnt0_row = res.antGeometryCfg.vAnt0_row;
    inst.antGeometryCfg_vAnt0_col = res.antGeometryCfg.vAnt0_col;
    inst.antGeometryCfg_vAnt1_row = res.antGeometryCfg.vAnt1_row;
    inst.antGeometryCfg_vAnt1_col = res.antGeometryCfg.vAnt1_col;
    inst.antGeometryCfg_vAnt2_row = res.antGeometryCfg.vAnt2_row;
    inst.antGeometryCfg_vAnt2_col = res.antGeometryCfg.vAnt2_col;
    inst.antGeometryCfg_vAnt3_row = res.antGeometryCfg.vAnt3_row;
    inst.antGeometryCfg_vAnt3_col = res.antGeometryCfg.vAnt3_col;
    inst.antGeometryCfg_vAnt4_row = res.antGeometryCfg.vAnt4_row;
    inst.antGeometryCfg_vAnt4_col = res.antGeometryCfg.vAnt4_col;
    inst.antGeometryCfg_vAnt5_row = res.antGeometryCfg.vAnt5_row;
    inst.antGeometryCfg_vAnt5_col = res.antGeometryCfg.vAnt5_col;
    inst.antGeometryCfg_antDistX = res.antGeometryCfg.antDistX;
    inst.antGeometryCfg_antDistZ = res.antGeometryCfg.antDistZ;
    
    inst.guiMonitor_pointCloud = res.guiMonitor.pointCloud;
    inst.guiMonitor_rangeProfile = res.guiMonitor.rangeProfile;
    inst.guiMonitor_noiseProfile = res.guiMonitor.noiseProfile;
    inst.guiMonitor_rangeAzimuthHeatMap = res.guiMonitor.rangeAzimuthHeatMap;
    inst.guiMonitor_rangeDopplerHeatMap = res.guiMonitor.rangeDopplerHeatMap;
    inst.guiMonitor_statsInfo = res.guiMonitor.statsInfo;
    inst.guiMonitor_presenceInfo = res.guiMonitor.presenceInfo;
    inst.guiMonitor_adcSamples = res.guiMonitor.adcSamples;
    inst.guiMonitor_trackerInfo = res.guiMonitor.trackerInfo;
    inst.guiMonitor_microDopplerInfo = res.guiMonitor.microDopplerInfo;
    inst.guiMonitor_classifierInfo = res.guiMonitor.classifierInfo;
    inst.guiMonitor_quickEvalInfo = res.guiMonitor.quickEvalInfo;
    if("pointCloudAntennaSymbols" in res.guiMonitor)
    {
        inst.guiMonitor_pointCloudAntennaSymbols = res.guiMonitor.pointCloudAntennaSymbols;
    }

    if("steerVecCorr" in res) {
    inst.steerVecCorr_enableAntSymbGen = res.steerVecCorr.enableAntSymbGen;
    inst.steerVecCorr_enableSteeringVectorCorrection = res.steerVecCorr.enableSteeringVectorCorrection;
    inst.steerVecCorr_enableAngleInterpolation = res.steerVecCorr.enableAngleInterpolation;
    }

    if("rangeCompCfg" in res) {
    inst.rangeCompCfg_enable = res.rangeCompCfg.enabled;
    if("compensationRange" in res.rangeCompCfg)
    { inst.rangeCompCfg_detectionRangeIdx = res.rangeCompCfg.compensationRange; }
    if("compensationSNR" in res.rangeCompCfg)
    { inst.rangeCompCfg_detectionSNR    = res.rangeCompCfg.compensationSNR; }
    if("minimumCompensationDistance" in res.rangeCompCfg)
    { inst.rangeCompCfg_minCompRange = res.rangeCompCfg.minimumCompensationDistance; }
    if("maximumCompensationDistance" in res.rangeCompCfg)
    { inst.rangeCompCfg_maxCompRange = res.rangeCompCfg.maximumCompensationDistance; }
    if("minimumCompensationAngle" in res.rangeCompCfg)
    { inst.rangeCompCfg_minCompAngle1 = res.rangeCompCfg.minimumCompensationAngle; }
    if("maximumCompensationAngle" in res.rangeCompCfg)
    { inst.rangeCompCfg_maxCompAngle1 = res.rangeCompCfg.maximumCompensationAngle; }
    if("secondaryMinimumCompensationAngle" in res.rangeCompCfg)
    { inst.rangeCompCfg_minCompAngle2 = res.rangeCompCfg.secondaryMinimumCompensationAngle; }
    if("secondaryMaximumCompensationAngle" in res.rangeCompCfg)
    { inst.rangeCompCfg_maxCompAngle2 = res.rangeCompCfg.secondaryMaximumCompensationAngle; }
    if("secondaryCompensationSNRDelta" in res.rangeCompCfg)
    { inst.rangeCompCfg_snrDropfromAngle1ToAngle2 = res.rangeCompCfg.secondaryCompensationSNRDelta; }
    }

    inst.sigProcChainCfg_azimuthFftSize = res.sigProcChainCfg.azimuthFftSize;
    inst.sigProcChainCfg_elevationFftSize = res.sigProcChainCfg.elevationFftSize;
    inst.sigProcChainCfg_motDetMode = res.sigProcChainCfg.motDetMode;
    inst.sigProcChainCfg_coherentDoppler = res.sigProcChainCfg.coherentDoppler;
    inst.sigProcChainCfg_numFrmPerMinorMotProc = res.sigProcChainCfg.numFrmPerMinorMotProc;
    inst.sigProcChainCfg_numMinorMotionChirpsPerFrame = res.sigProcChainCfg.numMinorMotionChirpsPerFrame;
    inst.sigProcChainCfg_forceMinorMotionVelocityToZero = res.sigProcChainCfg.forceMinorMotionVelocityToZero;
    inst.sigProcChainCfg_minorMotionVelocityInclusionThr = res.sigProcChainCfg.minorMotionVelocityInclusionThr;
    
    if("dopElevDimReductOrder" in res.sigProcChainCfg)
    {
        inst.sigProcChainCfg_dopElevDimReductOrder = res.sigProcChainCfg.dopElevDimReductOrder;
    }

    inst.cfarCfg_averageMode = res.cfarCfg.averageMode;
    inst.cfarCfg_winLen = res.cfarCfg.winLen;
    inst.cfarCfg_guardLen = res.cfarCfg.guardLen;
    inst.cfarCfg_noiseDiv = res.cfarCfg.noiseDiv;
    inst.cfarCfg_cyclicMode = res.cfarCfg.cyclicMode;
    inst.cfarCfg_thresholdScale = res.cfarCfg.thresholdScale;
    inst.cfarCfg_peakGroupingEn = res.cfarCfg.peakGroupingEn;
    inst.cfarCfg_sideLobeThreshold = res.cfarCfg.sideLobeThreshold;
    inst.cfarCfg_enableLocalMaxRange = res.cfarCfg.enableLocalMaxRange;
    inst.cfarCfg_enableLocalMaxAzimuth = res.cfarCfg.enableLocalMaxAzimuth;
    inst.cfarCfg_interpolateRange = res.cfarCfg.interpolateRange;
    inst.cfarCfg_interpolateAzimuth = res.cfarCfg.interpolateAzimuth;

    if("lookUpTableCorrectAzimuthDom" in res.cfarCfg)
    {
        inst.cfarCfg_lookUpTableCorrectAzimuthDom = res.cfarCfg.lookUpTableCorrectAzimuthDom;
    }

    inst.aoaFovCfg_minAzimuthDeg = res.aoaFovCfg.minAzimuthDeg;
    inst.aoaFovCfg_maxAzimuthDeg = res.aoaFovCfg.maxAzimuthDeg;
    inst.aoaFovCfg_minElevationDeg = res.aoaFovCfg.minElevationDeg;
    inst.aoaFovCfg_maxElevationDeg = res.aoaFovCfg.maxElevationDeg;

    inst.rangeSelCfg_minMeters = res.rangeSelCfg.minMeters;
    inst.rangeSelCfg_maxMeters = res.rangeSelCfg.maxMeters;

    inst.clutterRemoval_enabled = res.clutterRemoval.enabled;

    inst.compRangeBiasAndRxChanPhase_rangeBias = res.compRangeBiasAndRxChanPhase.rangeBias;

    inst.lowPowerCfg_enable = res.lowPowerCfg.enable;

    inst.factoryCalibCfg_saveEnable = res.factoryCalibCfg.saveEnable;
    inst.factoryCalibCfg_restoreEnable = res.factoryCalibCfg.restoreEnable;
    inst.factoryCalibCfg_rxGain = res.factoryCalibCfg.rxGain;
    inst.factoryCalibCfg_txBackoff = res.factoryCalibCfg.txBackoff;
    inst.factoryCalibCfg_flashOffset = res.factoryCalibCfg.flashOffset;
    
    if(inst.MPD_ENABLE == "1") {
        if("mpdBoundaryBox" in res) {
            let mpdArrayLen = res.mpdBoundaryBox.length;
            let lastBoundaryBox = res.mpdBoundaryBox[mpdArrayLen-1].cmd.split(" ");
            
            inst.mpd_boundary_box_num = lastBoundaryBox[1];
            
            let mpdNulStr = "mpdBoundaryBox 0 0 0 0 0 0 0";
            
            switch(inst.mpd_boundary_box_num) {
                case "1":
                    inst.mpd_boundary_box1 = res.mpdBoundaryBox[0].cmd;
                    inst.mpd_boundary_box2 = mpdNulStr;
                    inst.mpd_boundary_box3 = mpdNulStr;
                    inst.mpd_boundary_box4 = mpdNulStr;
                    inst.mpd_boundary_box5 = mpdNulStr;
                    inst.mpd_boundary_box6 = mpdNulStr;
                    inst.mpd_boundary_box7 = mpdNulStr;
                    inst.mpd_boundary_box8 = mpdNulStr;
                    inst.mpd_boundary_box9 = mpdNulStr;
                    inst.mpd_boundary_box10 = mpdNulStr;
                    inst.mpd_boundary_box11 = mpdNulStr;
                    inst.mpd_boundary_box12 = mpdNulStr;
                    break;
                case "2":
                    inst.mpd_boundary_box1 = res.mpdBoundaryBox[0].cmd;
                    inst.mpd_boundary_box2 = res.mpdBoundaryBox[1].cmd;
                    inst.mpd_boundary_box3 = mpdNulStr;
                    inst.mpd_boundary_box4 = mpdNulStr;
                    inst.mpd_boundary_box5 = mpdNulStr;
                    inst.mpd_boundary_box6 = mpdNulStr;
                    inst.mpd_boundary_box7 = mpdNulStr;
                    inst.mpd_boundary_box8 = mpdNulStr;
                    inst.mpd_boundary_box9 = mpdNulStr;
                    inst.mpd_boundary_box10 = mpdNulStr;
                    inst.mpd_boundary_box11 = mpdNulStr;
                    inst.mpd_boundary_box12 = mpdNulStr;
                    break;
                case "3":
                    inst.mpd_boundary_box1 = res.mpdBoundaryBox[0].cmd;
                    inst.mpd_boundary_box2 = res.mpdBoundaryBox[1].cmd;
                    inst.mpd_boundary_box3 = res.mpdBoundaryBox[2].cmd;
                    inst.mpd_boundary_box4 = mpdNulStr;
                    inst.mpd_boundary_box5 = mpdNulStr;
                    inst.mpd_boundary_box6 = mpdNulStr;
                    inst.mpd_boundary_box7 = mpdNulStr;
                    inst.mpd_boundary_box8 = mpdNulStr;
                    inst.mpd_boundary_box9 = mpdNulStr;
                    inst.mpd_boundary_box10 = mpdNulStr;
                    inst.mpd_boundary_box11 = mpdNulStr;
                    inst.mpd_boundary_box12 = mpdNulStr;
                    break;
                case "4":
                    inst.mpd_boundary_box1 = res.mpdBoundaryBox[0].cmd;
                    inst.mpd_boundary_box2 = res.mpdBoundaryBox[1].cmd;
                    inst.mpd_boundary_box3 = res.mpdBoundaryBox[2].cmd;
                    inst.mpd_boundary_box4 = res.mpdBoundaryBox[3].cmd;
                    inst.mpd_boundary_box5 = mpdNulStr;
                    inst.mpd_boundary_box6 = mpdNulStr;
                    inst.mpd_boundary_box7 = mpdNulStr;
                    inst.mpd_boundary_box8 = mpdNulStr;
                    inst.mpd_boundary_box9 = mpdNulStr;
                    inst.mpd_boundary_box10 = mpdNulStr;
                    inst.mpd_boundary_box11 = mpdNulStr;
                    inst.mpd_boundary_box12 = mpdNulStr;
                    break;
                case "5":
                    inst.mpd_boundary_box1 = res.mpdBoundaryBox[0].cmd;
                    inst.mpd_boundary_box2 = res.mpdBoundaryBox[1].cmd;
                    inst.mpd_boundary_box3 = res.mpdBoundaryBox[2].cmd;
                    inst.mpd_boundary_box4 = res.mpdBoundaryBox[3].cmd;
                    inst.mpd_boundary_box5 = res.mpdBoundaryBox[4].cmd;
                    inst.mpd_boundary_box6 = mpdNulStr;
                    inst.mpd_boundary_box7 = mpdNulStr;
                    inst.mpd_boundary_box8 = mpdNulStr;
                    inst.mpd_boundary_box9 = mpdNulStr;
                    inst.mpd_boundary_box10 = mpdNulStr;
                    inst.mpd_boundary_box11 = mpdNulStr;
                    inst.mpd_boundary_box12 = mpdNulStr;
                    break;
                case "6":
                    inst.mpd_boundary_box1 = res.mpdBoundaryBox[0].cmd;
                    inst.mpd_boundary_box2 = res.mpdBoundaryBox[1].cmd;
                    inst.mpd_boundary_box3 = res.mpdBoundaryBox[2].cmd;
                    inst.mpd_boundary_box4 = res.mpdBoundaryBox[3].cmd;
                    inst.mpd_boundary_box5 = res.mpdBoundaryBox[4].cmd;
                    inst.mpd_boundary_box6 = res.mpdBoundaryBox[5].cmd;
                    inst.mpd_boundary_box7 = mpdNulStr;
                    inst.mpd_boundary_box8 = mpdNulStr;
                    inst.mpd_boundary_box9 = mpdNulStr;
                    inst.mpd_boundary_box10 = mpdNulStr;
                    inst.mpd_boundary_box11 = mpdNulStr;
                    inst.mpd_boundary_box12 = mpdNulStr;
                    break;
                case "7":
                    inst.mpd_boundary_box1 = res.mpdBoundaryBox[0].cmd;
                    inst.mpd_boundary_box2 = res.mpdBoundaryBox[1].cmd;
                    inst.mpd_boundary_box3 = res.mpdBoundaryBox[2].cmd;
                    inst.mpd_boundary_box4 = res.mpdBoundaryBox[3].cmd;
                    inst.mpd_boundary_box5 = res.mpdBoundaryBox[4].cmd;
                    inst.mpd_boundary_box6 = res.mpdBoundaryBox[5].cmd;
                    inst.mpd_boundary_box7 = res.mpdBoundaryBox[6].cmd;
                    inst.mpd_boundary_box8 = mpdNulStr;
                    inst.mpd_boundary_box9 = mpdNulStr;
                    inst.mpd_boundary_box10 = mpdNulStr;
                    inst.mpd_boundary_box11 = mpdNulStr;
                    inst.mpd_boundary_box12 = mpdNulStr;
                    break;
                case "8":
                    inst.mpd_boundary_box1 = res.mpdBoundaryBox[0].cmd;
                    inst.mpd_boundary_box2 = res.mpdBoundaryBox[1].cmd;
                    inst.mpd_boundary_box3 = res.mpdBoundaryBox[2].cmd;
                    inst.mpd_boundary_box4 = res.mpdBoundaryBox[3].cmd;
                    inst.mpd_boundary_box5 = res.mpdBoundaryBox[4].cmd;
                    inst.mpd_boundary_box6 = res.mpdBoundaryBox[5].cmd;
                    inst.mpd_boundary_box7 = res.mpdBoundaryBox[6].cmd;
                    inst.mpd_boundary_box8 = res.mpdBoundaryBox[7].cmd;
                    inst.mpd_boundary_box9 = mpdNulStr;
                    inst.mpd_boundary_box10 = mpdNulStr;
                    inst.mpd_boundary_box11 = mpdNulStr;
                    inst.mpd_boundary_box12 = mpdNulStr;
                    break;
                case "9":
                    inst.mpd_boundary_box1 = res.mpdBoundaryBox[0].cmd;
                    inst.mpd_boundary_box2 = res.mpdBoundaryBox[1].cmd;
                    inst.mpd_boundary_box3 = res.mpdBoundaryBox[2].cmd;
                    inst.mpd_boundary_box4 = res.mpdBoundaryBox[3].cmd;
                    inst.mpd_boundary_box5 = res.mpdBoundaryBox[4].cmd;
                    inst.mpd_boundary_box6 = res.mpdBoundaryBox[5].cmd;
                    inst.mpd_boundary_box7 = res.mpdBoundaryBox[6].cmd;
                    inst.mpd_boundary_box8 = res.mpdBoundaryBox[7].cmd;
                    inst.mpd_boundary_box9 = res.mpdBoundaryBox[8].cmd;
                    inst.mpd_boundary_box10 = mpdNulStr;
                    inst.mpd_boundary_box11 = mpdNulStr;
                    inst.mpd_boundary_box12 = mpdNulStr;
                    break;
                case "10":
                    inst.mpd_boundary_box1 = res.mpdBoundaryBox[0].cmd;
                    inst.mpd_boundary_box2 = res.mpdBoundaryBox[1].cmd;
                    inst.mpd_boundary_box3 = res.mpdBoundaryBox[2].cmd;
                    inst.mpd_boundary_box4 = res.mpdBoundaryBox[3].cmd;
                    inst.mpd_boundary_box5 = res.mpdBoundaryBox[4].cmd;
                    inst.mpd_boundary_box6 = res.mpdBoundaryBox[5].cmd;
                    inst.mpd_boundary_box7 = res.mpdBoundaryBox[6].cmd;
                    inst.mpd_boundary_box8 = res.mpdBoundaryBox[7].cmd;
                    inst.mpd_boundary_box9 = res.mpdBoundaryBox[8].cmd;
                    inst.mpd_boundary_box10 = res.mpdBoundaryBox[9].cmd;
                    inst.mpd_boundary_box11 = mpdNulStr;
                    inst.mpd_boundary_box12 = mpdNulStr;
                    break;
                case "11":
                    inst.mpd_boundary_box1 = res.mpdBoundaryBox[0].cmd;
                    inst.mpd_boundary_box2 = res.mpdBoundaryBox[1].cmd;
                    inst.mpd_boundary_box3 = res.mpdBoundaryBox[2].cmd;
                    inst.mpd_boundary_box4 = res.mpdBoundaryBox[3].cmd;
                    inst.mpd_boundary_box5 = res.mpdBoundaryBox[4].cmd;
                    inst.mpd_boundary_box6 = res.mpdBoundaryBox[5].cmd;
                    inst.mpd_boundary_box7 = res.mpdBoundaryBox[6].cmd;
                    inst.mpd_boundary_box8 = res.mpdBoundaryBox[7].cmd;
                    inst.mpd_boundary_box9 = res.mpdBoundaryBox[8].cmd;
                    inst.mpd_boundary_box10 = res.mpdBoundaryBox[9].cmd;
                    inst.mpd_boundary_box11 = res.mpdBoundaryBox[10].cmd;
                    inst.mpd_boundary_box12 = mpdNulStr;
                    break;
                case "12":
                    inst.mpd_boundary_box1 = res.mpdBoundaryBox[0].cmd;
                    inst.mpd_boundary_box2 = res.mpdBoundaryBox[1].cmd;
                    inst.mpd_boundary_box3 = res.mpdBoundaryBox[2].cmd;
                    inst.mpd_boundary_box4 = res.mpdBoundaryBox[3].cmd;
                    inst.mpd_boundary_box5 = res.mpdBoundaryBox[4].cmd;
                    inst.mpd_boundary_box6 = res.mpdBoundaryBox[5].cmd;
                    inst.mpd_boundary_box7 = res.mpdBoundaryBox[6].cmd;
                    inst.mpd_boundary_box8 = res.mpdBoundaryBox[7].cmd;
                    inst.mpd_boundary_box9 = res.mpdBoundaryBox[8].cmd;
                    inst.mpd_boundary_box10 = res.mpdBoundaryBox[9].cmd;
                    inst.mpd_boundary_box11 = res.mpdBoundaryBox[10].cmd;
                    inst.mpd_boundary_box12 = res.mpdBoundaryBox[11].cmd;
                    break;
                default:
                    inst.mpd_boundary_box_num = 0;
                    inst.fname = "ERROR PARSING CONFIG: Too many boxes, max supported is 12";
                    return;
            }
        }

        if("minorStateCfg" in res) {
            inst.minorStateCfg_en = "1";
            inst.minorStateCfg_point_thre1 = res.minorStateCfg.pointthre1;
            inst.minorStateCfg_point_thre2 = res.minorStateCfg.pointthre2;
            inst.minorStateCfg_snr_thre2 = res.minorStateCfg.snrthre2;
            inst.minorStateCfg_pointhist_thre1 = res.minorStateCfg.pointhistthre1;
            inst.minorStateCfg_pointhist_thre2 = res.minorStateCfg.pointhistthre2;
            inst.minorStateCfg_snrhist_thre2 = res.minorStateCfg.snrhistthre2;
            inst.minorStateCfg_histbuff_size = res.minorStateCfg.histbuffsize;
            inst.minorStateCfg_exit_thre = res.minorStateCfg.exitthre;
        }

        if("majorStateCfg" in res) {
            inst.majorStateCfg_en = "1";
            inst.majorStateCfg_point_thre1 = res.majorStateCfg.pointthre1;
            inst.majorStateCfg_point_thre2 = res.majorStateCfg.pointthre2;
            inst.majorStateCfg_snr_thre2 = res.majorStateCfg.snrthre2;
            inst.majorStateCfg_pointhist_thre1 = res.majorStateCfg.pointhistthre1;
            inst.majorStateCfg_pointhist_thre2 = res.majorStateCfg.pointhistthre2;
            inst.majorStateCfg_snrhist_thre2 = res.majorStateCfg.snrhistthre2;
            inst.majorStateCfg_histbuff_size = res.majorStateCfg.histbuffsize;
            inst.majorStateCfg_exit_thre = res.majorStateCfg.exitthre;
        }

        if("sensorposition" in res) {
            inst.sensorPosition_x = res.sensorposition.x;
            inst.sensorPosition_y = res.sensorposition.y;
            inst.sensorPosition_z = res.sensorposition.z;
            inst.sensorPosition_az = res.sensorposition.az;
            inst.sensorPosition_el = res.sensorposition.el;
        }

        if("clusterCfg" in res) {
            inst.clusterCfg_enable = res.clusterCfg.enable;
            inst.clusterCfg_radius = res.clusterCfg.radius;
            inst.clusterCfg_minpoints = res.clusterCfg.minpoints;
        }
    }

    inst.sensorStart_frameTrigMode = res.sensorStart.frameTrigMode;
    inst.sensorStart_chirpStartSigLbEn = res.sensorStart.chirpStartSigLbEn;
    inst.sensorStart_frameLivMonEn = res.sensorStart.frameLivMonEn;
    inst.sensorStart_frameTrigTimerVal = res.sensorStart.frameTrigTimerVal;
}
let mpd_demo_module_name = "/demo/mpd_demo/mpd_demo";

let mpd_demo_module = {
    displayName: "MPD_DEMO",
    templates: {
        "/demo/demo/cli_mpd_demo_config.h.xdt": {
            cli_mpd_demo_config: "/demo/mpd_demo/templates/cli_mpd_demo_config.h.xdt",
        },
    },

    defaultInstanceName: "CONFIG_MPD_DEMO",
    maxInstances: 1,
    config:  getConfigurables(),
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    getInstanceConfig,
    getInterfaceName,
};

function getModule()
{
    let module = mpd_demo_module;
    return module;
}

exports = getModule();
