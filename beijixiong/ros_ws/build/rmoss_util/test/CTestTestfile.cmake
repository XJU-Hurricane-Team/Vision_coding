# CMake generated Testfile for 
# Source directory: /home/chairman/working/ros2/beijixiong/ros_ws/src/rmoss_core/rmoss_util/test
# Build directory: /home/chairman/working/ros2/beijixiong/ros_ws/build/rmoss_util/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_url_resolve "/home/chairman/venv/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/chairman/working/ros2/beijixiong/ros_ws/build/rmoss_util/test_results/rmoss_util/test_url_resolve.gtest.xml" "--package-name" "rmoss_util" "--output-file" "/home/chairman/working/ros2/beijixiong/ros_ws/build/rmoss_util/ament_cmake_gtest/test_url_resolve.txt" "--command" "/home/chairman/working/ros2/beijixiong/ros_ws/build/rmoss_util/test/test_url_resolve" "--gtest_output=xml:/home/chairman/working/ros2/beijixiong/ros_ws/build/rmoss_util/test_results/rmoss_util/test_url_resolve.gtest.xml")
set_tests_properties(test_url_resolve PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/chairman/working/ros2/beijixiong/ros_ws/build/rmoss_util/test/test_url_resolve" TIMEOUT "60" WORKING_DIRECTORY "/home/chairman/working/ros2/beijixiong/ros_ws/build/rmoss_util/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/chairman/working/ros2/beijixiong/ros_ws/src/rmoss_core/rmoss_util/test/CMakeLists.txt;3;ament_add_gtest;/home/chairman/working/ros2/beijixiong/ros_ws/src/rmoss_core/rmoss_util/test/CMakeLists.txt;0;")
subdirs("../gtest")
