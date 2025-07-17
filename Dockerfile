FROM nvcr.io/nvidia/isaac-sim:2023.1.1

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO humble
ENV ROS_VERSION 2
ENV TERM=xterm
ENV NVIDIA_DRIVER_CAPABILITIES compute,graphics,utility, display
ENV XDG_RUNTIME_DIR=/run/user/1000
ENV VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
ENV VK_LAYER_PATH=/usr/share/vulkan/explicit_layer.d
ENV ISAACSIM_PATH=/isaac-sim
ENV ISAACSIM_PYTHON_EXE=/isaac-sim/python.sh
ENV LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Install ROS
RUN apt update && apt install locales software-properties-common curl -y
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    locale
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && \
    echo $VERSION_CODENAME)_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb
RUN apt update && apt upgrade -y && apt install ros-humble-desktop ros-dev-tools -y
RUN rosdep init && rosdep update
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install conda
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh && \
    /bin/sh /tmp/miniconda.sh -b -p /opt/conda && \
    rm /tmp/miniconda.sh
ENV PATH /opt/conda/bin:$PATH

RUN conda init bash
RUN conda config --set auto_activate false

RUN conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r

RUN conda install python=3.10.18

# Copy local directory to docker
WORKDIR /
COPY . go2_omniverse

WORKDIR /go2_omniverse/IsaacLab

# Create a symbolic link to ISAACSIM_PATH
RUN ln -s /isaac-sim _isaac_sim

# Run orbit.sh to create the conda environment
RUN conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r
RUN ./orbit.sh --conda

RUN apt install cmake build-essential

# Install additional components
#RUN sed -i '/^gymnasium/!b; s/gymnasium==.*/gymnasium==0.28.1/; t; a gymnasium==0.28.1' docs/requirements.txt # Jank fix
RUN conda run -n orbit ./orbit.sh --install
RUN conda run -n orbit pip install git+https://github.com/leggedrobotics/rsl_rl.git@v2.0.0#egg=rsl_rl
RUN conda run -n orbit pip install "gymnasium==0.28.1" "numpy<1.25,>=1.21" "usd-core<24.00,>=21.11" "protobuf==3.19.6"

WORKDIR /

# Copy the Unitree_L1.json config file
RUN mkdir -p go2_omniverse/IsaacLab/source/data/sensors/lidar && \
    cp /go2_omniverse/Isaac_sim/Unitree/Unitree_L1.json go2_omniverse/IsaacLab/source/data/sensors/lidar/Unitree_L1.json

# Copy material files
RUN mkdir -p go2_omniverse/IsaacLab/source/data/material_files && \
    cp -r /isaac-sim/data/material_files/* go2_omniverse/IsaacLab/source/data/material_files/

# Build ROS workspaces
RUN pip3 install "empy==3.3.4" "lark" "catkin_pkg" "numpy<2"
WORKDIR /go2_omniverse
RUN /bin/bash -c \
    "source /opt/ros/humble/setup.bash && \
    cd IsaacSim-ros_workspaces/${ROS_DISTRO}_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build"
RUN echo "source /go2_omniverse/IsaacSim-ros_workspaces/${ROS_DISTRO}_ws/install/setup.bash" >> ~/.bashrc

RUN /bin/bash -c \
    "source /opt/ros/humble/setup.bash && \
    cd go2_omniverse_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build"
RUN echo "source /go2_omniverse/go2_omniverse_ws/install/setup.bash" >> ~/.bashrc

# Vulkan
RUN echo '{ \
    "file_format_version" : "1.0.1", \
    "ICD": { \
        "library_path": "libGLX_nvidia.so.0", \
        "api_version" : "1.4.303" \
    } \
}' > /usr/share/vulkan/icd.d/nvidia_icd.json

RUN apt install -y \
    libvulkan1 \
    vulkan-tools \
    mesa-vulkan-drivers \
    && rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]
