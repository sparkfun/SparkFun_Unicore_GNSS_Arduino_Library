#!/bin/bash
#
# check.sh
#    Script to verify example builds
########################################################################
set -e
#set -o verbose
#set -o xtrace

cd examples/Example1_PositionVelocityTime
make clean
make
make clean

cd ../Example2_DirectConnect
make clean
make
make clean

cd ../Example3_ECEFAndStats
make clean
make
make clean

cd ../Example4_EnableNMEA_5Hz
make clean
make
make clean

cd ../Example5_EnableRTCM
make clean
make
make clean

cd ../Example6_AverageBase
make clean
make
make clean

cd ../Example7_FixedBase
make clean
make
make clean

cd ../Example8_SetConstellations
make clean
make
make clean

cd ../Example9_SignalElevation
make clean
make
make clean

cd ../Example10_SetRoverMode
make clean
make
make clean

cd ../Example11_UsbNMEA
make clean
make
make clean

cd ../Example12_FactoryReset
make clean
make
make clean

cd ../Example13_SendCommand
make clean
make
make clean

# SPARTN_Test
cd ../Example14_SetPPS
make clean
make
make clean

# UBLOX_Test
cd ../Example15_QueryDevice
make clean
make
make clean

# Unicore_Binary_Test
cd ../Example16_PollForValidRTCMMessages
make clean
make
make clean

# Unicore_Hash_Test
cd ../Example17_Parser
make clean
make
make clean

# User_Parser_Test
cd ../Example18_CheckConfig
make clean
make
make clean

# User_Parser_Test
cd ../Example19_Enable_Galileo_HAS
make clean
make
make clean

# User_Parser_Test
cd ../Example20_NTRIP_Client
make clean
make
make clean

# User_Parser_Test
cd ../Example21_NTRIP_Server
make clean
make
make clean

# User_Parser_Test
cd ../Example22_SetAntennaDescription
make clean
make
make clean

# Return to origin directory
cd ../..
