sudo docker ps -a
read -n 1 -s -p "Press any key to continue..."
sudo docker start ros2gpu
sudo docker exec -it ros2gpu /bin/bash
