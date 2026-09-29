cmake_minimum_required(VERSION 3.10)
project(rover_rmf_maps)

find_package(ament_cmake REQUIRED)

# Building maps (traffic-editor format) and their floor plans, generated from rover_world.sdf by
# scripts/generate_rover_world.py. The nav graphs are generated here at build time, like
# rmf_demos_maps does, but without Gazebo worlds or model downloads: the rover's own Gazebo
# world stays the simulation.
file(GLOB_RECURSE building_paths "maps/*.building.yaml")

foreach(path ${building_paths})
  string(REGEX REPLACE "\\.[^.]*\\.[^.]*$" "" no_extension_path ${path})
  string(REGEX MATCH "[^\/]+$" world_name ${no_extension_path})
  set(output_dir ${CMAKE_CURRENT_BINARY_DIR}/maps/${world_name})
  set(nav_graph ${output_dir}/nav_graphs/0.yaml)

  add_custom_command(
    OUTPUT ${nav_graph}
    COMMAND ros2 run rmf_building_map_tools building_map_generator nav ${path} ${output_dir}/nav_graphs
    DEPENDS ${path}
    COMMENT "Generating the ${world_name} nav graph")
  add_custom_target(generate_${world_name}_nav_graphs ALL DEPENDS ${nav_graph})

  install(DIRECTORY ${output_dir}/nav_graphs DESTINATION share/${PROJECT_NAME}/maps/${world_name})
endforeach()

install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # scripts/import_rover_map.py's logic (rover_map_import.py): PGM/thresholds, transform, lanes.
  ament_add_pytest_test(test_rover_map_import test/test_rover_map_import.py)
  # Map sources (directory and ssh, with a stand-in ssh), write_site, the preview.
  ament_add_pytest_test(test_site_io test/test_site_io.py)
endif()

ament_package()
