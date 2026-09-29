cmake_minimum_required(VERSION 3.10)
project(rover_rmf_fleet_adapter)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/fleet_adapter DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Domain + application with fakes: no ROS graph, no RMF, no broker.
  ament_add_pytest_test(test_vda_messages test/unit/test_vda_messages.py)
  ament_add_pytest_test(test_robot_status test/unit/test_robot_status.py)
  ament_add_pytest_test(test_command_tracker test/unit/test_command_tracker.py)
  ament_add_pytest_test(test_robot_session test/unit/test_robot_session.py)
  add_test(
    NAME check_domain_purity
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_domain_purity.sh ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
