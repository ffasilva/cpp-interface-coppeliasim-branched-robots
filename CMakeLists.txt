CMAKE_MINIMUM_REQUIRED(VERSION 3.4)

if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

project(dmc-interface-coppeliasim)
set (CMAKE_CXX_STANDARD 11)

INCLUDE_DIRECTORIES(include)

list(APPEND CMAKE_PREFIX_PATH "~/GitHub/dynamic-modular-composition-cpp/build/Desktop_Qt_6_7_2-Debug")
find_package(dqdynamics REQUIRED)

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

    src/interfaces/coppeliasim/legacy_api/robots/LBR4pVrepRobotDynamics.cpp
    )

INSTALL(TARGETS dmc-interface-coppeliasim
# https://stackoverflow.com/questions/21592361/cmake-install-is-not-installing-libraries-on-windows
    RUNTIME DESTINATION "bin"
    LIBRARY DESTINATION "lib"
    ARCHIVE DESTINATION "lib"
    PUBLIC_HEADER DESTINATION "include/dmc-interface-coppeliasim"
    PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

if(UNIX AND NOT APPLE)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME}
        dqdynamics::dqdynamics)
endif()

# The directive above does not seem to work on macOS
if(APPLE)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME}
        -ldqdynamics)
endif()

# The directive above does not seem to work on Windows
if(WIN32)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME}
        ${DQDYNAMICS_LIB})
endif()

################################################################
# INSTALL HEADERS IN SUBFOLDERS
################################################################

# interfaces headers
INSTALL(FILES
    include/dmc-interface-coppeliasim/interfaces/coppeliasim/legacy_api/DQ_BranchedCoppeliaSimLegacyRobot.h

    DESTINATION "include/dmc-interface-coppeliasim/interfaces/coppeliasim/legacy_api")

# robots headers
INSTALL(FILES
    include/dmc-interface-coppeliasim/interfaces/coppeliasim/legacy_api/robots/LBR4pVrepRobotDynamics.h

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
    src/interfaces/coppeliasim/legacy_api/robots/LBR4pVrepRobotDynamics.cpp

    DESTINATION "src/interfaces/coppeliasim/legacy_api/robots")


# add_executable(DQ_BranchedCoppeliaSimLegacyRobot
#     src/interfaces/coppeliasim/legacy_api/DQ_BranchedCoppeliaSimLegacyRobot.cpp
#     )

# TARGET_LINK_LIBRARIES(DQ_BranchedCoppeliaSimLegacyRobot
#     dqdynamics::dqdynamics
# )

################################################################
# MAKE dmc-interface-coppeliasim AVAILABLE AS AN EXTERNAL LIBRARY
################################################################

# Then add an alias of your library to enable target syntax
add_library(dmc-interface-coppeliasim::dmc-interface-coppeliasim ALIAS dmc-interface-coppeliasim)

# Then, include gnuinstalldir to get the platform's standard directories:
include(GNUInstallDirs)

# Then, carefully add your include directories. All of your `target_include_directories` must look like this
target_include_directories(dmc-interface-coppeliasim PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # include directory in your build tree
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>       # include directory when installed
)

# Now, create the install script. We install all header files under `include/dmc-interface-coppeliasim` to install them in `<prefix>/include/dmc-interface-coppeliasim`:
install(
    DIRECTORY include/dmc-interface-coppeliasim
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# We add `dmc-interface-coppeliasim` target into the export set.
# The export set will contain all targets to be imported by the other project.
# It also installs the library to the install script so they are installed:
install(TARGETS dmc-interface-coppeliasim EXPORT dmc-interface-coppeliasim-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION "include/dmc-interface-coppeliasim"
)

# Now, we install the export set. This will generate a CMake file exporting all the target for other projects to use:
install(EXPORT dmc-interface-coppeliasim-targets
    DESTINATION "${CMAKE_INSTALL_DATADIR}/cmake/dmc-interface-coppeliasim"
    NAMESPACE dmc-interface-coppeliasim::
)

# Now, we also export the current buildtree. Other project will be able to import the project directly from a build dir:
configure_file(cmake/dmc-interface-coppeliasim-config.cmake dmc-interface-coppeliasim-config.cmake COPYONLY)
export(
    EXPORT dmc-interface-coppeliasim-targets
    NAMESPACE dmc-interface-coppeliasim::
    FILE "${PROJECT_BINARY_DIR}/dmc-interface-coppeliasim-targets.cmake"
)

# The file we created earlier:
install(
    FILES cmake/dmc-interface-coppeliasim-config.cmake
    DESTINATION "${CMAKE_INSTALL_DATADIR}/cmake/dmc-interface-coppeliasim"
)
