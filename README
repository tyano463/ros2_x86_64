```
ros2 pkg create --build-type ament_cmake ros2_cpp --dependencies rclcpp std_msgs
```

```diff
diff --git a/CMakeLists.txt b/CMakeLists.txt
index 0a9a296..13e633e 100644
--- a/CMakeLists.txt
+++ b/CMakeLists.txt
@@ -10,6 +10,22 @@ find_package(ament_cmake REQUIRED)
 find_package(rclcpp REQUIRED)
 find_package(std_msgs REQUIRED)
 
+add_executable(duplex_node src/duplex_node.cpp)
+
+target_include_directories(duplex_node PUBLIC
+  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
+  $<INSTALL_INTERFACE:include>
+  ${rclcpp_INCLUDE_DIRS}
+  ${std_msgs_INCLUDE_DIRS}
+)
+
+target_link_libraries(duplex_node
+  rclcpp::rclcpp
+  ${std_msgs_TARGETS}
+)
+
+install(TARGETS duplex_node DESTINATION lib/${PROJECT_NAME})
+
 if(BUILD_TESTING)
   find_package(ament_lint_auto REQUIRED)
   # the following line skips the linter which checks for copyrights
```

```
colcon build --packages-select ros2_cpp
```

```
. install/setup.bash
ros2 run ros2_cpp duplex_node
```



