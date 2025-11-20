var EXAMPLES_MMWDEMOS =
[
    [ "Motion and Presence Detection OOB Demo", "MOTION_AND_PRESENCE_DETECTION_DEMO.html", [
      [ "Purpose and Scope", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md752", null ],
      [ "Supported Combinations", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#MOTION_AND_PRESENCE_DETECTION_DEMO_COMBOS", null ],
      [ "Supported Features", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md753", null ],
      [ "Using SysConfig", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md754", [
        [ "Custom Demo Code Options", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md755", [
          [ "Steps to enable/disable the supported features", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md756", null ],
          [ "CLI Removal Feature", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md757", null ]
        ] ]
      ] ],
      [ "Motion Detection Demo Setup", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md758", null ],
      [ "MIMO Modulation Schemes", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md759", null ],
      [ "Chirp Configuration Modes", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md760", null ],
      [ "Antenna Geometry", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md761", null ],
      [ "Signal Processing Chain", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md762", [
        [ "Motion Detection Modes", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md763", null ],
        [ "Dynamic Reconfig", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md764", null ]
      ] ],
      [ "Variable SNR detection thresholds", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#RANGE_ANGLE_SNR", [
        [ "SNR vs Range - Feature Description", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md765", null ],
        [ "SNR vs Angle - Feature Description", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md766", null ]
      ] ],
      [ "Rx Channel Gain/Offset Measurement and Compensation", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md767", [
        [ "Measurement Procedure Implementation", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md768", null ]
      ] ],
      [ "System execution flow", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md769", [
        [ "Initialization and Configuration", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md770", null ],
        [ "Steady state – low power mode disabled", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md771", null ],
        [ "Steady state – low power mode enabled", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md772", null ]
      ] ],
      [ "Task Model", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md773", [
        [ "Main Task", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md774", null ],
        [ "CLI Task", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md775", null ],
        [ "DPC Task", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md776", null ],
        [ "UART Task", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md777", null ],
        [ "ADC read data Task", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md778", null ],
        [ "Power Management Task", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md779", null ],
        [ "Timing Diagram", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md780", null ]
      ] ],
      [ "Benchmarks", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md781", null ],
      [ "Configuration (.cfg) File Format (CLI INTERFACE)", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md782", [
        [ "Sensor front-end parameters", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md783", null ],
        [ "Detection layer parameters", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md784", null ],
        [ "Motion/presence detection layer", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md785", null ],
        [ "Tracking layer parameters", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md786", null ],
        [ "Classification layer parameters", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md787", null ],
        [ "Profile switching parameters", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md788", null ]
      ] ],
      [ "UART and Output to the Host", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md789", [
        [ "Output TLV Description", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md790", [
          [ "Frame Header Structure", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md791", null ],
          [ "TLV Structure", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md792", null ],
          [ "Point Cloud TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md793", [
            [ "Floating point format", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md794", null ],
            [ "Fix-point format", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md797", null ]
          ] ],
          [ "Range Profile TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md798", null ],
          [ "Detection heatmap TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md799", null ],
          [ "Presence Detection TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md800", null ],
          [ "Stats TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md801", null ],
          [ "Group tracker data TLVs", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md802", [
            [ "List of tracking objects TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md803", null ],
            [ "Group Tracker indices array TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md804", null ]
          ] ],
          [ "Micro-Doppler TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md805", null ],
          [ "Features TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md806", null ],
          [ "Classifier Output TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md807", null ],
          [ "Quick-Eval TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md808", null ],
          [ "Rx channel compensation measurement output TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md809", null ],
          [ "Antenna symbols TLV", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md810", null ]
        ] ]
      ] ],
      [ "Low Power Configuration (lowPowerCfg = 1)", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md811", [
        [ "Flow Diagram", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md812", null ],
        [ "Threshold and Latency time duration for Power module", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md813", null ]
      ] ],
      [ "Raw ADC data streaming", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md814", [
        [ "Steps for DCA based streaming", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md815", [
          [ "DCA1000EVM Setup", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md816", null ],
          [ "Power on/off sequence", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md817", null ],
          [ "Steps to Perform Raw ADC Data Streaming", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md818", null ],
          [ "Validation of the ADC data", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md819", null ]
        ] ],
        [ "SPI based streaming of Raw ADC Data", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md820", [
          [ "Building the demo with this feature enabled", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md821", null ],
          [ "Steps to Perform Raw ADC Data Streaming on xWRL1432 FCCSP device", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md822", null ],
          [ "Validation of the ADC data", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md823", null ]
        ] ]
      ] ],
      [ "Steering Vector Tool Usage", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md824", [
        [ "Running the Tool", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md825", null ],
        [ "Input Parameters", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md826", null ],
        [ "Output", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md827", null ],
        [ "Examples", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md828", [
          [ "Example 1: Basic Usage", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md829", null ],
          [ "Example 2: Custom Spacing", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md830", null ]
        ] ],
        [ "Troubleshooting", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md831", null ]
      ] ],
      [ "Running in test mode", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md832", null ],
      [ "Steps to Run Monitors", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md833", null ],
      [ "Steps to Run the Example", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md834", null ],
      [ "Power Measurements with INA228", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md835", null ],
      [ "Memory Usage", "MOTION_AND_PRESENCE_DETECTION_DEMO.html#autotoc_md836", null ]
    ] ],
    [ "Gesture Recognition Demo", "GESTURE_RECOGNITION_DEMO.html", [
      [ "Supported Combinations", "GESTURE_RECOGNITION_DEMO.html#GESTURE_RECOGNITION_DEMO_COMBOS", null ],
      [ "Using SDK with SysConfig", "GESTURE_RECOGNITION_DEMO.html#autotoc_md841", null ],
      [ "Steps to Run the Example", "GESTURE_RECOGNITION_DEMO.html#autotoc_md842", null ],
      [ "Configuration (.cfg) File Format (CLI INTERFACE)", "GESTURE_RECOGNITION_DEMO.html#autotoc_md843", [
        [ "Sensor front-end parameters", "GESTURE_RECOGNITION_DEMO.html#autotoc_md844", null ],
        [ "Signal Processing layer parameters", "GESTURE_RECOGNITION_DEMO.html#autotoc_md845", null ],
        [ "Presence Detect configuration parameters", "GESTURE_RECOGNITION_DEMO.html#autotoc_md846", null ]
      ] ],
      [ "Flow Diagram in Low Power Configuration (lowPowerCfg = 1)", "GESTURE_RECOGNITION_DEMO.html#autotoc_md847", null ]
    ] ],
    [ "Mmwave Demo", "MMWAVE_DEMO.html", [
      [ "Purpose and Scope", "MMWAVE_DEMO.html#autotoc_md848", null ],
      [ "Supported Combinations", "MMWAVE_DEMO.html#MMWAVE_DEMO_COMBOS", null ],
      [ "Supported Features", "MMWAVE_DEMO.html#autotoc_md849", null ],
      [ "Using SysConfig", "MMWAVE_DEMO.html#autotoc_md850", [
        [ "Custom Demo Code Options", "MMWAVE_DEMO.html#autotoc_md851", [
          [ "Steps to enable/disable the supported features", "MMWAVE_DEMO.html#autotoc_md852", null ],
          [ "CLI Removal Feature", "MMWAVE_DEMO.html#autotoc_md853", null ]
        ] ]
      ] ],
      [ "MMWAVE Demo Setup", "MMWAVE_DEMO.html#autotoc_md854", null ],
      [ "MIMO Modulation Schemes", "MMWAVE_DEMO.html#autotoc_md855", null ],
      [ "Antenna Geometry", "MMWAVE_DEMO.html#autotoc_md856", null ],
      [ "Signal Processing Chain", "MMWAVE_DEMO.html#autotoc_md857", [
        [ "Data Compression", "MMWAVE_DEMO.html#autotoc_md858", [
          [ "Range processing DPU", "MMWAVE_DEMO.html#autotoc_md859", null ],
          [ "Doppler Processing DPU", "MMWAVE_DEMO.html#autotoc_md860", null ],
          [ "AoA2D DPU", "MMWAVE_DEMO.html#autotoc_md861", null ]
        ] ]
      ] ],
      [ "Rx Channel Gain/Offset Measurement and Compensation", "MMWAVE_DEMO.html#autotoc_md862", [
        [ "Measurement Procedure Implementation", "MMWAVE_DEMO.html#autotoc_md863", null ]
      ] ],
      [ "System execution flow", "MMWAVE_DEMO.html#autotoc_md864", null ],
      [ "Task Model", "MMWAVE_DEMO.html#autotoc_md865", [
        [ "Main Task", "MMWAVE_DEMO.html#autotoc_md866", null ],
        [ "CLI Task", "MMWAVE_DEMO.html#autotoc_md867", null ],
        [ "DPC Task", "MMWAVE_DEMO.html#autotoc_md868", null ],
        [ "UART Task", "MMWAVE_DEMO.html#autotoc_md869", null ],
        [ "ADC read data Task", "MMWAVE_DEMO.html#autotoc_md870", null ],
        [ "Power Management Task", "MMWAVE_DEMO.html#autotoc_md871", null ],
        [ "Timing Diagram", "MMWAVE_DEMO.html#autotoc_md872", null ]
      ] ],
      [ "Memory Usage", "MMWAVE_DEMO.html#autotoc_md873", null ],
      [ "Benchmarks", "MMWAVE_DEMO.html#autotoc_md874", null ],
      [ "Configuration (.cfg) File Format (CLI INTERFACE)", "MMWAVE_DEMO.html#autotoc_md875", [
        [ "Sensor front-end parameters", "MMWAVE_DEMO.html#autotoc_md876", null ],
        [ "Detection layer parameters", "MMWAVE_DEMO.html#autotoc_md877", null ]
      ] ],
      [ "UART and Output to the Host", "MMWAVE_DEMO.html#autotoc_md878", [
        [ "Output TLV Description", "MMWAVE_DEMO.html#autotoc_md879", [
          [ "Frame Header Structure", "MMWAVE_DEMO.html#autotoc_md880", null ],
          [ "TLV Structure", "MMWAVE_DEMO.html#autotoc_md881", null ],
          [ "Point Cloud TLV", "MMWAVE_DEMO.html#autotoc_md882", [
            [ "Floating point format", "MMWAVE_DEMO.html#autotoc_md883", null ],
            [ "Fix-point format", "MMWAVE_DEMO.html#autotoc_md886", null ]
          ] ],
          [ "Range Profile TLV", "MMWAVE_DEMO.html#autotoc_md887", null ],
          [ "Range-Doppler Heatmap TLV", "MMWAVE_DEMO.html#autotoc_md888", null ],
          [ "Stats TLV", "MMWAVE_DEMO.html#autotoc_md889", null ],
          [ "Rx channel compensation measurement output TLV", "MMWAVE_DEMO.html#autotoc_md890", null ]
        ] ]
      ] ],
      [ "Low Power Configuration (lowPowerCfg = 1)", "MMWAVE_DEMO.html#autotoc_md891", [
        [ "Flow Diagram", "MMWAVE_DEMO.html#autotoc_md892", null ],
        [ "Threshold and Latency time duration for Power module", "MMWAVE_DEMO.html#autotoc_md893", null ]
      ] ],
      [ "Raw ADC data streaming", "MMWAVE_DEMO.html#autotoc_md894", [
        [ "Steps for DCA based streaming", "MMWAVE_DEMO.html#autotoc_md895", [
          [ "DCA1000EVM Setup", "MMWAVE_DEMO.html#autotoc_md896", null ],
          [ "Power on/off sequence", "MMWAVE_DEMO.html#autotoc_md897", null ],
          [ "Steps to Perform Raw ADC Data Streaming", "MMWAVE_DEMO.html#autotoc_md898", null ],
          [ "Validation of the ADC data", "MMWAVE_DEMO.html#autotoc_md899", null ]
        ] ],
        [ "SPI based streaming of Raw ADC Data", "MMWAVE_DEMO.html#autotoc_md900", [
          [ "Building the demo with this feature enabled", "MMWAVE_DEMO.html#autotoc_md901", null ],
          [ "Steps to Perform Raw ADC Data Streaming on xWRL1432 FCCSP device:", "MMWAVE_DEMO.html#autotoc_md902", null ],
          [ "Validation of the ADC data", "MMWAVE_DEMO.html#autotoc_md903", null ]
        ] ]
      ] ],
      [ "Running in test mode", "MMWAVE_DEMO.html#autotoc_md904", null ],
      [ "Steps to Run Monitors", "MMWAVE_DEMO.html#autotoc_md905", null ],
      [ "Steps to Run the Example", "MMWAVE_DEMO.html#autotoc_md906", null ],
      [ "Power Measurements with INA228", "MMWAVE_DEMO.html#autotoc_md907", null ]
    ] ],
    [ "Monitors", "MONITORS.html", [
      [ "Steps to run Monitors", "MONITORS.html#autotoc_md837", null ],
      [ "Steps to run Live Monitors", "MONITORS.html#autotoc_md838", null ],
      [ "Configuration (.cfg) File Format (CLI INTERFACE)", "MONITORS.html#autotoc_md839", [
        [ "Monitor parameters", "MONITORS.html#autotoc_md840", null ]
      ] ]
    ] ]
];