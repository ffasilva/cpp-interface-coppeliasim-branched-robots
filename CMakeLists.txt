CMAKE_MINIMUM_REQUIRED(VERSION 3.4)

if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

project(dqdynamics-interface-coppeliasim)
set (CMAKE_CXX_STANDARD 14)

INCLUDE_DIRECTORIES(include)

if(UNIX AND NOT APPLE)
    FIND_PACKAGE(Eigen3 REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR}) #Writing this way is more OS agnostic
    ADD_COMPILE_OPTIONS(-O3 -Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
endif()

if(APPLE)
    INCLUDE_DIRECTORIES(
        /usr/local/include/
        /usr/local/include/eigen3
        # Most recent versions of brew install here
        /opt/homebrew/include
        /opt/homebrew/include/eigen3)
    ADD_COMPILE_OPTIONS(-O3 -Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
    # The library is installed here when using the regular cmake ., make, sudo make install
    LINK_DIRECTORIES(
        /usr/local/lib/
        /opt/homebrew/lib/
        )
endif()

if(WIN32)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    FIND_LIBRARY(DQDYNAMICS_LIB NAMES dqdynamics::dqdynamics)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES) #For windows to find M_PI
    FIND_PACKAGE(Eigen3 CONFIG REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
endif()

###############################################################
# DEFINE AND INSTALL LIBRARY AND INCLUDE FOLDER
################################################################

ADD_LIBRARY(dqdynamics-interface-coppeliasim SHARED
    # src/dqdynamics/interfaces/coppeliasim/legacy_api/DQ_BranchedCoppeliaSimLegacyRobot.cpp

    # src/dqdynamics/interfaces/coppeliasim/legacy_api/robots/LBR4pCoppeliaSimLegacyRobot.cpp

    src/dqdynamics/interfaces/coppeliasim/zmq/DQ_BranchedCoppeliaSimZMQRobot.cpp

    src/dqdynamics/interfaces/coppeliasim/zmq/robots/B1LegCoppeliaSimZMQ.cpp
    src/dqdynamics/interfaces/coppeliasim/zmq/robots/HexapodLegCoppeliaSimZMQ.cpp
    src/dqdynamics/interfaces/coppeliasim/zmq/robots/LBR4pCoppeliaSimZMQRobot.cpp
    src/dqdynamics/interfaces/coppeliasim/zmq/robots/FreeFlyingCoppeliaSimZMQRobot.cpp

    src/dqdynamics/interfaces/coppeliasim/zmq/robots/B1CoppeliaSimZMQRobot.cpp
    src/dqdynamics/interfaces/coppeliasim/zmq/robots/HexapodCoppeliaSimZMQRobot.cpp
    src/dqdynamics/interfaces/coppeliasim/zmq/robots/BM_TRO2023CoppeliaSimZMQRobot.cpp

    src/dqdynamics/utils/math.cpp
    src/dqdynamics/utils/NumericalDifferentiation.cpp
    src/dqdynamics/utils/ButterworthFilter.cpp
)

# Links with required libraries
if(UNIX)
    target_link_libraries(${PROJECT_NAME}
        dqdynamics

        dqrobotics
        dqrobotics-interface-coppeliasim-zmq
    )
endif()

INSTALL(TARGETS dqdynamics-interface-coppeliasim
# https://stackoverflow.com/questions/21592361/cmake-install-is-not-installing-libraries-on-windows
    RUNTIME DESTINATION "bin"
    LIBRARY DESTINATION "lib"
    ARCHIVE DESTINATION "lib"
    PUBLIC_HEADER DESTINATION "include/dqdynamics-interface-coppeliasim"
    PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

################################################################
# INSTALL HEADERS IN SUBFOLDERS
################################################################

# # Legacy API interfaces headers
# INSTALL(FILES
#     include/dqdynamics/interfaces/coppeliasim/legacy_api/DQ_BranchedCoppeliaSimLegacyRobot.h

#     DESTINATION "include/dqdynamics/interfaces/coppeliasim/legacy_api")

# # Legacy API robots headers
# INSTALL(FILES
#     include/dqdynamics/interfaces/coppeliasim/legacy_api/robots/LBR4pCoppeliaSimLegacyRobot.h

#     DESTINATION "include/dqdynamics/interfaces/coppeliasim/legacy_api/robots")

# ZMQ interfaces headers
INSTALL(FILES
    include/dqdynamics/interfaces/coppeliasim/zmq/DQ_BranchedCoppeliaSimZMQRobot.h

    DESTINATION "include/dqdynamics/interfaces/coppeliasim/zmq")

# ZMQ robots headers
INSTALL(FILES
    include/dqdynamics/interfaces/coppeliasim/zmq/robots/B1LegCoppeliaSimZMQ.h
    include/dqdynamics/interfaces/coppeliasim/zmq/robots/HexapodLegCoppeliaSimZMQ.h
    include/dqdynamics/interfaces/coppeliasim/zmq/robots/LBR4pCoppeliaSimZMQRobot.h
    include/dqdynamics/interfaces/coppeliasim/zmq/robots/FreeFlyingCoppeliaSimZMQRobot.h

    include/dqdynamics/interfaces/coppeliasim/zmq/robots/B1CoppeliaSimZMQRobot.h
    include/dqdynamics/interfaces/coppeliasim/zmq/robots/HexapodCoppeliaSimZMQRobot.h
    include/dqdynamics/interfaces/coppeliasim/zmq/robots/BM_TRO2023CoppeliaSimZMQRobot.h

    DESTINATION "include/dqdynamics/interfaces/coppeliasim/zmq/robots")

# utils headers
INSTALL(FILES
    include/dqdynamics/utils/math.h
    include/dqdynamics/utils/NumericalDifferentiation.h
    include/dqdynamics/utils/ButterworthFilter.h

    DESTINATION "include/dqdynamics/utils")

################################################################
# INSTALL SOURCE FILES (So that the debugger can find them)
################################################################

# # Legacy API base folder
# INSTALL(FILES
#     src/dqdynamics/interfaces/coppeliasim/legacy_api/DQ_BranchedCoppeliaSimLegacyRobot.cpp

#     DESTINATION "src/dqdynamics/interfaces/coppeliasim/legacy_api")

# # Legacy API robots folder
# INSTALL(FILES
#     src/dqdynamics/interfaces/coppeliasim/legacy_api/robots/LBR4pCoppeliaSimLegacyRobot.cpp

#     DESTINATION "src/dqdynamics/interfaces/coppeliasim/legacy_api/robots")

# ZMQ base folder
INSTALL(FILES
    src/dqdynamics/interfaces/coppeliasim/zmq/DQ_BranchedCoppeliaSimZMQRobot.cpp

    DESTINATION "src/dqdynamics/interfaces/coppeliasim/zmq")

# ZMQ robots folder
INSTALL(FILES
    src/dqdynamics/interfaces/coppeliasim/zmq/robots/B1LegCoppeliaSimZMQ.cpp
    src/dqdynamics/interfaces/coppeliasim/zmq/robots/HexapodLegCoppeliaSimZMQ.cpp
    src/dqdynamics/interfaces/coppeliasim/zmq/robots/LBR4pCoppeliaSimZMQRobot.cpp
    src/dqdynamics/interfaces/coppeliasim/zmq/robots/FreeFlyingCoppeliaSimZMQRobot.cpp

    src/dqdynamics/interfaces/coppeliasim/zmq/robots/B1CoppeliaSimZMQRobot.cpp
    src/dqdynamics/interfaces/coppeliasim/zmq/robots/HexapodCoppeliaSimZMQRobot.cpp
    src/dqdynamics/interfaces/coppeliasim/zmq/robots/BM_TRO2023CoppeliaSimZMQRobot.cpp

    DESTINATION "src/dqdynamics/interfaces/coppeliasim/zmq/robots")

# utils folder
INSTALL(FILES
    src/dqdynamics/utils/math.cpp
    src/dqdynamics/utils/NumericalDifferentiation.cpp
    src/dqdynamics/utils/ButterworthFilter.cpp

    DESTINATION "src/dqdynamics/utils")
