#ifndef SOCKETCAN_BRIDGE_CONFIGURATION_VARS_H_
#define SOCKETCAN_BRIDGE_CONFIGURATION_VARS_H_

namespace socketcan_bridge {

// Radar Configuration 0x200

const int RADARCFG_MAXDISTANCE_VALID = 1; // Allow change of maximum distance 0-1
const int RADARCFG_SENSORID_VALID = 1; // Allow change of sensor Id 0-1
const int RADARCFG_RADARPOWER_VALID = 1; // Allow change in radar output power
const int RADARCFG_OUTPUTTYPE_VALID = 1; // Allow change of output type
const int RADARCFG_SENDQUALITY_VALID = 1; // Allow change of quality option
const int RADARCFG_SENDEXTINFO_VALID = 1; // Allow change of extended information
const int RADARCFG_SORTINDEX_VALID = 1; // Allow change of sorting index
const int RADARCFG_STOREINNVM_VALID = 1; // Allow storing to non-volatile memory

const int RADARCFG_MAXDISTANCE = 250; // Maximum distance of far scan. Standard range: 96-260
const int RADARCFG_SENSORID = 0; // Sensor Id 0-7
const int RADARCFG_OUTPUTTYPE = 2; // Output type 0 none - 1 objects - 2 clusters
const int RADARCFG_RADARPOWER = 3; // Transmitted radar power 0 Standard - 3 -9dB Tx gain

const int RADARCFG_CTRLRELAY_VALID = 0; // Allow change of relay control output
const int RADARCFG_CTRLRELAY = 0; // Relay control message is sent if true and collision detection is activated
const int RADARCFG_SENDQUALITY = 1; // Send quality information
const int RADARCFG_SENDEXTINFO = 1; // Send extended information
const int RADARCFG_SORTINDEX = 1; // Sorting index 0 no sorting - 1 sort by range - 2 sort by RCS
const int RADARCFG_STOREINNVM = 1; // Store the current configuration to non-volatile memory

const int RADARCFG_RCS_THRESHOLD_VALID = 1; // Allow change of RCS threshold option
const int RADARCFG_RCS_THRESHOLD = 1; // Set the sensibility of cluster detection 0 standard - 1 high sensibility

// Cluster and Object Filter Configuration 0x202

const int FILTERCFG_VALID = 1; // Allow change of filter configuration
const int FILTERCFG_ACTIVE = 1; // Activate filter configuration
const int FILTERCFG_INDEX = 5; // Multiplexor to specify which filter criterion to configure
const int FILTERCFG_TYPE = 0; // Choose between objects and clusters 0 cluster - 1 object
// Index 0. Clusters and objects
const double FILTERCFG_MIN_NOFOBJ = 0; // Number of clusters or objects 0 - 4095
const double FILTERCFG_MAX_NOFOBJ = 50;
const double FILTERCFG_RES_NOFOBJ = 1;
// Index 1. Clusters and objects
const double FILTERCFG_MIN_DISTANCE = 0; // Radial distance 0 - 409.5 m 
const double FILTERCFG_MAX_DISTANCE = 30;
const double FILTERCFG_RES_DISTANCE = 0.1;
// Index 2. Clusters and objects
const double FILTERCFG_MIN_AZIMUTH = -50; // Azimuth angle -50 - 52.375 deg
const double FILTERCFG_MAX_AZIMUTH = 52.375;
const double FILTERCFG_MINIMUM_AZIMUTH = -50;
const double FILTERCFG_RES_AZIMUTH = 0.025;
// Index 3. Clusters and objects
const double FILTERCFG_MIN_VRELONCOME = 0; // Radial velocity in sensor line-of-sight of oncoming obj/clus 
const double FILTERCFG_MAX_VRELONCOME = 128.993; // 0 - 128.993 m/s
const double FILTERCFG_RES_VRELONCOME = 0.0315;
// Index 4. Clusters and objects
const double FILTERCFG_MIN_VRELDEPART = 0; // Radial velocity in sensor line-of-sight of departing obj/clus
const double FILTERCFG_MAX_VRELDEPART = 128.993; // 0 - 128.993 m/s
const double FILTERCFG_RES_VRELDEPART= 0.0315;
// Index 5. Clusters and objects
const double FILTERCFG_MIN_RCS = -20; // Radar Cross Section -50 - 52.375 dBm2
const double FILTERCFG_MAX_RCS = 52.375;
const double FILTERCFG_MINIMUM_RCS = -50;
const double FILTERCFG_RES_RCS = 0.025;
// Index 6. Objects
const double FILTERCFG_MIN_LIFETIME = 0; // Life time since first detection 0 - 409.5 s
const double FILTERCFG_MAX_LIFETIME = 409.5;
const double FILTERCFG_RES_LIFETIME = 0.1;
// Index 7. Objects
const double FILTERCFG_MIN_SIZE = 0; // Area object size 0 - 102.375 m2
const double FILTERCFG_MAX_SIZE = 102.375;
const double FILTERCFG_RES_SIZE = 0.025;
// Index 8. Objects
const double FILTERCFG_MIN_PROBEXISTS = 3; // Probability of existence
const double FILTERCFG_MAX_PROBEXISTS = 7; //  0 0% - 1 25 % - 2 50 % - 3 75 % - 4 90 % - 5 99 % - 6 99.9 % - 7 100 %
const double FILTERCFG_RES_PROBEXISTS = 1;
// Index 9. Objects
const double FILTERCFG_MIN_Y = -409.5; // Lateral distance -409.5 - 409.5 m
const double FILTERCFG_MAX_Y = 409.5;
const double FILTERCFG_MINIMUM_Y = -409.5;
const double FILTERCFG_RES_Y = 0.2;
// Index 10. Objects
const double FILTERCFG_MIN_X = -10; // Longitudinal distance -500 - 1138.2 m
const double FILTERCFG_MAX_X = 260;
const double FILTERCFG_MINIMUM_X = -500;
const double FILTERCFG_RES_X = 0.2;
// Index 11. Objects
const double FILTERCFG_MIN_VYRIGHTLEFT = 0; // Lateral velocity component for right-left moving obj
const double FILTERCFG_MAX_VYRIGHTLEFT = 128.993; // 0 - 128.993 m/s
const double FILTERCFG_RES_VYRIGHTLEFT = 0.0315;
// Index 12. Objects
const double FILTERCFG_MIN_VXONCOME = 0; // Longitudinal velocity component for oncoming moving obj
const double FILTERCFG_MAX_VXONCOME = 128.993; // 0 - 128.993 m/s
const double FILTERCFG_RES_VXONCOME = 0.0315;
// Index 13. Objects
const double FILTERCFG_MIN_VYLEFTRIGHT = 0; // Lateral velocity component for left-right moving obj
const double FILTERCFG_MAX_VYLEFTRIGHT = 128.993; // 0 - 128.993 m/s
const double FILTERCFG_RES_VYLEFTRIGHT = 0.0315;
// Index 14. Objects
const double FILTERCFG_MIN_VXDEPART = 0; // Longitudinal velocity component for departing moving ob
const double FILTERCFG_MAX_VXDEPART = 128.993; // 0 - 128.993 m/s
const double FILTERCFG_RES_VXDEPART = 0.0315;

// Collision Detection Configuration 0x400

const int COLLDETCFG_WARNINGRESET = 0; // Reset currently active warnings of all regions
const int COLLDETCFG_ACTIVATION = 0; // Activate collision detection function
const int COLLDETCFG_MINTIME_VALID = 0; // Allow change of time parameter
const int COLLDETCFG_CLEARREGIONS = 0; // Clear all region configurations
const double COLLDETCFG_MINTIME = 0; // Minimum time an object needs to be detected inside the region 0 - 25.5 s
const double COLLDETCFG_RESTIME = 0.1;

// Collision Detection Region Configuration 0x401

const int COLLDETREGCFG_ACTIVATION = 1; // Activate current region
const int COLLDETREGCFG_COORDINATES_VALID = 1; // Allow change of current region coordinates
const int COLLDETREGCFG_REGIONID = 0; // Id of current region to configure
const double COLLDETREGCFG_POINT1X = 0; // Longitudinal position of first point -500 - 1138.2 m
const double COLLDETREGCFG_POINT1Y = -1; // Lateral position of first point -204.6 - 204.8 m
const double COLLDETREGCFG_POINT2X = 1; // Longitudinal position of second point -500 - 1138.2 m
const double COLLDETREGCFG_POINT2Y = 1; // Lateral position of second point -204.6 - 204.8 m
const double COLLDETREGCFG_POINTRES = 0.2;

}  // namespace socketcan_bridge

#endif
