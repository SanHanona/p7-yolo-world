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
./build_isaac.sh
```

You will be prompted to give a username and password 
```bash
usernam: $oauthtoken
Pass: ask or create a account and key: https://docs.nvidia.com/ngc/gpu-cloud/ngc-user-guide/index.html#generating-api-key 
```

### Build Llama and whisper
Navigate to isaac_sim folder 
```bash
cd llama
```
Run the following to build 
```bash
./build_llama.sh
```

## Run with docker-compose 

Running the **container on local machine** you will need to give permission to X server before running the container: 
```bash
xhost +local:root
```
Running the **container from a host** e.g. ssh you might need to run from the host: 
```bash
xhost +local:docker
```

You can run the Docker containers with docker compose - This will execute entrypoints for all containers which starts the system

Run both containers in detached mode as dev container (running in the background - to not run in background run without '-d')
```bash
docker compose -f docker-compose-dev up -d
```

The isaac-sim dev container only mounts the isaac-sim/humble_ws folder 

### Verify container
Verify the containers are runnning: 
```bash
docker compose ps
```

You should see all containers running 

## Inside the Container

### Isaac-sim 
The Isaac-sim container executes a runheadless script. You should be able to access the stream through the streaming client using the ip of the host. 

### YOLO-World
Access the container with: 
```bash
docker exec -it yolo-demo-dev /bin/bash
```

Once inside the container, navigate to the `demo/` directory and execute various YOLO test examples using Python 3:
```bash
cd demo/
python3 yolo_test.py
```

# ROS2 
All containers have ros2 humble install and should be able to communicate. For trouble-shooting ros2 issues test the containers can communicate. First access both containers in two different terminals. 

## navigation demo 
To run the navigation with rvis: 
- open the streaming client 
-> Isaac Examples --> ROS2 --> Navigation --> Carter Navigation
-> press play

- In a terminal exec the isaac sim container
```bash
docker exec -it isaac-sim-dev /bin/bash
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
docker exec -it yolo-demo-dev /bin/bash
```

- run the entrypoint
```bash
./yolo_entrypoint.sh
```

- In a new terminal exec the yolo world 
```bash
docker exec -it llama-dev /bin/bash
```

- run the entrypoint
```bash
./llama_entrypoint.sh
```




### debug
Run a listener and talker 
```bash
ros2 run demo_nodes_py talker
```
```bash
ros2 run demo_nodes_py listener
```


