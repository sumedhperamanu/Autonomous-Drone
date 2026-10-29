How to test my 5" sim (branch 5-inch-sim)
1) In Ubuntu (WSL), start a container with the Gazebo window enabled:
docker run -it --name fiveinch --network=host --shm-size=2g -e DISPLAY -e WAYLAND_DISPLAY -e XDG_RUNTIME_DIR -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg drone-sim:px4-1.17 bash
2) Inside the container (root@... prompt), paste:
git clone -b 5-inch-sim REPO_URL ~/drone && bash ~/drone/fiveinch-sim/fiveinch/install.sh ~/PX4-Autopilot && cd ~/PX4-Autopilot && make px4_sitl gz_fiveinch
3) At pxh>, type one line at a time:
commander takeoff
listener actuator_motors
commander land
Pass = hovers at about 2.5 m, all 4 motors about 0.35.
4) Lidar test: type shutdown, then:
PX4_GZ_WORLD=walls make px4_sitl gz_fiveinch_lidar_flow
At pxh>:
listener obstacle_distance
listener distance_sensor
listener sensor_optical_flow
Pass = 72 lidar distances (walls at about 450 cm), ground distance about 0.07, flow quality 255.
Next time: docker start -ai fiveinch, then start from the make line.
