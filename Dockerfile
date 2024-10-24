# Base image with CUDA support
FROM nvidia/cuda:12.1.0-devel-ubuntu22.04 AS base

# Set environment variables
ENV FORCE_CUDA="1" \
    MMCV_WITH_OPS=1 \
    DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

WORKDIR /yolo

# Install system dependencies including prerequisites for ROS2
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    libgl1-mesa-glx \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libglib2.0-0 \
    python3-dev \
    python3-wheel \
    -qqy x11-apps \
    curl \
    lsb-release \
    gnupg2 \
    locales \
    build-essential \
    git \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*


# Add ROS2 Humble apt repository
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-argcomplete \
    && rosdep init \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS2 environment variables
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install Python dependencies
FROM base AS python_deps

RUN pip3 install --upgrade pip wheel \
    && pip3 install --no-cache-dir torch==2.1.2 torchvision==0.16.2 torchaudio==2.1.2 --index-url https://download.pytorch.org/whl/cu121 \
    && pip3 install --no-cache-dir \
        gradio==4.16.0 \
        opencv-python==4.9.0.80 \
        supervision \
        mmengine==0.10.4 \
        setuptools \
        openmim \
        onnx \
        onnxsim \
    && mim install mmcv==2.1.0 \
    && mim install mmdet==3.3.0 \
    && pip3 install --no-cache-dir git+https://github.com/onuralpszr/mmyolo.git \
    && pip install -q inference-gpu[yolo-world]==0.9.12rc1 

# Clone and install YOLO-World
FROM python_deps AS yolo_world

COPY . /yolo

RUN pip3 install -e .[demo]

# Final stage
FROM yolo_world AS final

ARG MODEL="yolo_world_l_dual_vlpan_l2norm_2e-3_100e_4x8gpus_obj365v1_goldg_train_lvis_minival.py"
ARG WEIGHT="yolo_world_l_clip_base_dual_vlpan_2e-3adamw_32xb16_100e_o365_goldg_train_pretrained-0e566235.pth"

# Create weights directory and set permissions
RUN mkdir /weights/ \
    && chmod a+rwx /yolo/configs/*/*

# Optionally download weights (commented out by default)
RUN curl -o /weights/$WEIGHT -L https://huggingface.co/wondervictor/YOLO-World/resolve/main/$WEIGHT

# Set the default command
# Set the entrypoint to source ROS 2 automatically
# ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec \"$@\""]
