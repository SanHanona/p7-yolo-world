# p7-yolo-world + isaac sim container 

## Prerequisites

Before you begin, ensure you have the following installed:

- **Nvidia driver**: Version 560.35.03 has proven to work with this repo 
- **CUDA 12.1**: [Download CUDA 12.1](https://developer.nvidia.com/cuda-12-1-0-download-archive)
- **Nvidia Container Toolkit** (Linux only): Follow [this guide](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html#container-setup) (see step 3).
- **Nvidia omniverse streaming client**: https://docs.omniverse.nvidia.com/launcher/latest/it-managed-launcher/install_guide_linux.html 


## Build Instructions

To build the Docker image

### Build YOLO-World
Run the following command in your terminal:
```bash
./build.sh pretrain-l-clip
```

### Build Isaac-sim 
Navigate to isaac_sim folder 
```bash
cd isaac-sim
```
Run the following to build 
```bash
./build.sh
```

You will be prompted to give a username and password 
```bash
usernam: $oauthtoken
Pass: ask or create a account and key: https://docs.nvidia.com/ngc/gpu-cloud/ngc-user-guide/index.html#generating-api-key 
```

## Run with docker-compose 

You can run the Docker containers with docker compose 

Navigate to docker-compose
```bash
cd docker-compose
```

Run both containers in detached mode (running in the background)
```bash
docker compose up -d
```

### Verify container
Verify the containers are runnning: 
```bash
docker compose ps
```

You should see both containers running 

## Inside the Container

### Isaac-sim 
The Isaac-sim container executes a runheadless script. You should be able to access the stream through the streaming client using the ip of the host. 

### YOLO-World
Access the container with: 
```bash
docker exec -it yolo-demo /bin/bash
```

Once inside the container, navigate to the `demo/` directory and execute various YOLO test examples using Python 3:
```bash
cd demo/
python3 yolo_test.py
```

# ROS2 
Both containers have ros2 humble install and should be able to communicate. For trouble-shooting ros2 issues test the containers can communicate. First access both containers in two different terminals. 

## navigation demo 
To run the navigation with rvis: 
- open the streaming client 
-> Isaac Examples --> ROS2 --> Navigation --> Carter Navigation
-> press play

- In a terminal exec the isaac sim container
```bash
docker exec -it isaac-sim /bin/bash
```

- Source the installs 
```bash
source humble_ws/install/setup.bash
```

- Run rviz and all that 
```bash
ros2 launch carter_navigation carter_navigation.launch.py
```

- In a new terminal exec the yolo world 
```bash
docker exec -it yolo-demo /bin/bash
```

- Source the installs 
```bash
source ros_ws/install/setup.bash
```

- Run navigation and all that 
```bash
ros2 launch nav_goal_handler nav_goal_handler.launch.py
```

### debug
Run a listener and talker 
```bash
ros2 run demo_nodes_py talker
```
```bash
ros2 run demo_nodes_py listener
```


