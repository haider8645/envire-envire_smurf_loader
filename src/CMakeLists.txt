find_package(Boost COMPONENTS filesystem system thread)

rock_library(envire_smurf_loader
    SOURCES Model.cpp
    HEADERS Model.hpp
    DEPS_PKGCONFIG
        base-types
        configmaps
        base-lib
        urdfdom
        envire_core
        envire_base_types
    DEPS_PLAIN
        Boost_SYSTEM
        Boost_FILESYSTEM
        Boost_THREAD
)

FIND_PACKAGE(Boost COMPONENTS program_options filesystem REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

rock_executable(envire_smurf_loader_bin Main.cpp
    DEPS envire_smurf_loader)
TARGET_LINK_LIBRARIES(envire_smurf_loader_bin ${Boost_LIBRARIES})

