#!/bin/bash
sudo docker ps -a
read -n 1 -s -p  "Press any key to stop docker -> "
sudo docker stop ros2gpu
