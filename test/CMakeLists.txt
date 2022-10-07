find_package(ament_cmake_nose REQUIRED)
ament_add_nose_test(${PROJECT_NAME}_test_vesc_messages test_vesc_messages.py)
