
cmake_minimum_required(VERSION 3.5.1)

project(ACF_HS_Detect)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-mfpu=neon-vfpv4")

include_directories(
        /opt/opencv3.4.1/build/3rdparty/tbb/tbb-2018_U1/include
        /usr/local/include/
        /usr/include/
        /usr/include/arm-linux-gnueabihf/
)

add_executable(
        ACF_HS_Detect 
        main.cpp
        control/HumanInfrared.cpp
        control/InfraredRemote.cpp
        control/Relay.cpp
        control/LinpRemote.cpp
        low-level/convConst.cpp
        low-level/gradientMex.cpp
        low-level/rgbConvertMex.cpp
        acf/ACFDetector.cpp
        acf/ACFFeaturePyramid.cpp
        acf/Channel.cpp
        acf/ChannelFeatures.cpp
        acf/ColorChannel.cpp
        general/detection.cpp
        general/DetectionList.cpp
        general/NonMaximumSuppression.cpp
)
	
link_directories(
        /usr/local/lib/
        /usr/lib/
        /usr/lib/arm-linux-gnueabihf/
)

target_link_libraries(
        ACF_HS_Detect 
        opencv_world
        pthread 
        raspicam
        pigpio
        matio 
        tbb
)
