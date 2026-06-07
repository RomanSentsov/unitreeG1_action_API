cmake_minimum_required(VERSION 3.8)
project(service_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include ${ament_INCLUDE_DIRS})

# The current Modbus driver implements only these services.
set(SUPPORTED_HAND_SERVICES
  "srv/Setpos.srv"
  "srv/Setforce.srv"
  "srv/Setspeed.srv"
  "srv/Getposact.srv"
)

# Vendor API declarations kept for compatibility/reserved future support.
set(RESERVED_VENDOR_SERVICES
  "srv/Getcurrentact.srv"
  "srv/Geterror.srv"
  "srv/Getposset.srv"
  "srv/Getspeedset.srv"
  "srv/Getforceact.srv"
  "srv/Getforceset.srv"
  "srv/Getangleset.srv"
  "srv/Getangleact.srv"
  "srv/Gettemp.srv"
  "srv/Getstatus.srv"
  
  "srv/Setid.srv"
  "srv/Setreduratio.srv"
  "srv/Setclearerror.srv"
  "srv/Setresetpara.srv"
  "srv/Setdefaultspeed.srv"
  "srv/Setangle.srv"
  "srv/Setsaveflash.srv"
  "srv/Setdefaultforce.srv"
  "srv/Setforceclb.srv"
  "srv/Setcurrentlimit.srv"
  "srv/Setgestureno.srv"
)

# 使用 rosidl_generate_interfaces 来生成服务
rosidl_generate_interfaces(${PROJECT_NAME}
  ${SUPPORTED_HAND_SERVICES}
  ${RESERVED_VENDOR_SERVICES}
  DEPENDENCIES std_msgs
)

# 测试和代码检查
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 最后，调用 ament_package()
ament_package()
