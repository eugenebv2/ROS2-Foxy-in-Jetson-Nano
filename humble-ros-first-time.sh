sudo docker pull dustynv/ros:humble-ros-base-l4t-r32.7.1

sudo docker run --rm -it --runtime nvidia --network host --gpus all -e DISPLAY dustynv/ros:humble-ros-base-l4t-r32.7.1
