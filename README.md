# p7-yolo-world

## Prerequisites

Before you begin, ensure you have the following installed:

- **CUDA 12.1**: [Download CUDA 12.1](https://developer.nvidia.com/cuda-12-1-0-download-archive)
- **Nvidia Container Toolkit** (Linux only): Follow [this guide](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html#container-setup) (see step 3).
- **Nvidia Runtime for Docker** (Windows only): Refer to [this Stack Overflow thread](https://stackoverflow.com/questions/77323535/add-nvidia-runtime-to-docker-runtimes-on-windows-wsl) for instructions.

## Build Instructions

To build the Docker image, follow the instructions for your operating system:

### Linux
Run the following command in your terminal:
```bash
./build.sh pretrain-l-clip
```

## Running the Docker Container

You can run the Docker container in different modes based on your operating system:

### Linux
To run the Docker container, use:
```bash
./run_headless.sh pretrain-l-clip
`

## Inside the Container

Once inside the container, navigate to the `demo/` directory and execute various YOLO test examples using Python 3:
```bash
cd demo/
python3 yolo_test.py
```

