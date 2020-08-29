include_directories(${CMAKE_SOURCE_DIR})

# Descend into the mpga_phototaxis_loop_functions directory
add_subdirectory(mpga_loop_functions)

# If Qt+OpenGL dependencies were found, descend into these directories
if(ARGOS_COMPILE_QTOPENGL)
  add_subdirectory(zone_loop_functions)
endif(ARGOS_COMPILE_QTOPENGL)

