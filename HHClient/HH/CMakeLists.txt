cmake_minimum_required(VERSION 3.13)
project(HH)
add_executable(HH 
                src/main.cpp
                src/parkingLotGraph.cpp)

# Add the include directory to the target
target_include_directories(HH PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_include_directories(HH PUBLIC ${CMAKE_SOURCE_DIR}/ext/libcarla-install/include)
target_include_directories(HH PUBLIC ${CMAKE_SOURCE_DIR}/ext/libcarla-install/include/system)

# Link the libraries to the target
target_link_libraries(HH PRIVATE
    carla_client
    rpc
    Recast
    # Link the Detour libraries dynamically
    -Wl,-Bdynamic
    Detour
    DetourCrowd
    DetourTileCache
    # Link the other libraries dynamically
    -Wl,-Bdynamic
    png
    tiff
    jpeg
    # Link Boost library directly
    -Wl,-Bstatic
    boost_filesystem
    -Wl,-Bdynamic
    boost_program_options
    boost_python310
    boost_system
)

# Specify the directory containing Boost libraries
target_link_directories(HH PUBLIC ${CMAKE_SOURCE_DIR}/ext/libcarla-install/lib)

# Link the Boost library directly
target_link_libraries(HH PRIVATE ${CMAKE_SOURCE_DIR}/ext/libcarla-install/lib/libboost_atomic.so.1.80.0)

# Set the RPATH for the executable
set_target_properties(HH PROPERTIES
    INSTALL_RPATH "${CMAKE_SOURCE_DIR}/ext/libcarla-install/lib"
)
