CMAKE_MINIMUM_REQUIRED(VERSION 3.4)

if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

project(dmc-interface-coppeliasim)
set (CMAKE_CXX_STANDARD 11)

INCLUDE_DIRECTORIES(include)

if(UNIX AND NOT APPLE)
    FIND_PACKAGE(Eigen3 REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR}) #Writing this way is more OS agnostic
    ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
endif()

if(APPLE)
    INCLUDE_DIRECTORIES(
        /usr/local/include/
        /usr/local/include/eigen3
        # Most recent versions of brew install here
        /opt/homebrew/include
        /opt/homebrew/include/eigen3)
    ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
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

ADD_LIBRARY(dmc-interface-coppeliasim SHARED
    src/interfaces/coppeliasim/legacy_api/DQ_BranchedCoppeliaSimLegacyRobot.cpp

    src/interfaces/coppeliasim/legacy_api/robots/LBR4pCoppeliaSimLegacyRobot.cpp
    )

# Links with dqrobotics, otherwise Qt returns "Undefined symbols for architecture arm64" on Apple silicon
if(APPLE)
    target_link_libraries(${PROJECT_NAME}
        dqrobotics
    )
endif()

INSTALL(TARGETS dmc-interface-coppeliasim
# https://stackoverflow.com/questions/21592361/cmake-install-is-not-installing-libraries-on-windows
    RUNTIME DESTINATION "bin"
    LIBRARY DESTINATION "lib"
    ARCHIVE DESTINATION "lib"
    PUBLIC_HEADER DESTINATION "include/dmc-interface-coppeliasim"
    PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

################################################################
# INSTALL HEADERS IN SUBFOLDERS
################################################################

# interfaces headers
INSTALL(FILES
    include/dmc-interface-coppeliasim/interfaces/coppeliasim/legacy_api/DQ_BranchedCoppeliaSimLegacyRobot.h

    DESTINATION "include/dmc-interface-coppeliasim/interfaces/coppeliasim/legacy_api")

# robots headers
INSTALL(FILES
    include/dmc-interface-coppeliasim/interfaces/coppeliasim/legacy_api/robots/LBR4pCoppeliaSimLegacyRobot.h

    DESTINATION "include/dmc-interface-coppeliasim/interfaces/coppeliasim/legacy_api/robots")

################################################################
# INSTALL SOURCE FILES (So that the debugger can find them)
################################################################

# base folder
INSTALL(FILES
    src/interfaces/coppeliasim/legacy_api/DQ_BranchedCoppeliaSimLegacyRobot.cpp

    DESTINATION "src/interfaces/coppeliasim/legacy_api")

# robots folder
INSTALL(FILES
    src/interfaces/coppeliasim/legacy_api/robots/LBR4pCoppeliaSimLegacyRobot.cpp

    DESTINATION "src/interfaces/coppeliasim/legacy_api/robots")
