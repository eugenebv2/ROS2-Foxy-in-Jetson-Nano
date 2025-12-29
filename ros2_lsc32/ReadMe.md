# This code was generate by AI Copilot


# run theNode

// ros2 run your_package lsc32_controller --ros-args -p port:=/dev/ttyTHS1 -p baudrate:=115200

ros2 run arm_test lsc32_test_1 --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200


# Send a commend

ros2 topic pub /lsc32_cmd std_msgs/UInt16MultiArray "{data: [1, 500, 2, 600, 1000]}"

========================================================
arm_test Start the Service Node:

ros2 run your_package lsc32_multi_servo_service --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200

Call the service:

ros2 service call /move_servos your_package/srv/MultiServo \
"{ids: [1, 2], positions: [500, 600], time_ms: 1500}"

# Run the YAML Client

ros2 run your_package lsc32_yaml_service_client /path/to/servo_sequence.yaml


# Using default YAML path and serial settings
ros2 launch your_package lsc32_service_sequence.launch.py

# Using a custom YAML file and serial settings
ros2 launch your_package lsc32_service_sequence.launch.py \
    yaml_path:=/path/to/servo_sequence.yaml \
    port:=/dev/ttyAMA0 \
    baudrate:=57600

# Running the Multi-Servo Service with return
Start the service node:

ros2 run my_robot_interfaces lsc32_multi_service_node

# Call the service:

ros2 service list

ros2 service call /move_servos arm_servie/srv/MultiServo "{ids: [1, 2, 3], positions: [500, 600, 700], time_ms: 1500}"



# Launch the service 
ros2 launch arm_test lsc32_service_sequence.launch.py

# start send a sequence yaml servo control file and loop

ros2 run arm_test lsc32_yaml_service_client_loop /home/jetson/ros2_lc32/sg_yaml/servo_sequence.yaml 1
