include(CMakeFindDependencyMacro)

# write it like you find_package of your cmake scripts
find_dependency(Qt6 COMPONENTS Core Widgets REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/dmc-interface-coppeliasim-targets.cmake")