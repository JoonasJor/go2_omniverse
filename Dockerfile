FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO humble
ENV ROS_VERSION 2
ENV TZ=Europe/Helsinki
ENV TERM=xterm
ENV NVIDIA_DRIVER_CAPABILITIES compute,graphics,utility, display
ENV XDG_RUNTIME_DIR=/run/user/1000
ENV VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
ENV VK_LAYER_PATH=/usr/share/vulkan/explicit_layer.d

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
RUN echo ". /opt/conda/etc/profile.d/conda.sh" >> /etc/profile.d/conda.sh

# Create the conda environment
RUN conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r && \
    conda create -n go2_omniverse python=3.10 -y

# Install Isaac Sim
RUN conda run -n go2_omniverse pip install isaacsim[all]==4.5.0 --extra-index-url https://pypi.nvidia.com

# Install Isaac Lab
RUN git clone https://github.com/isaac-sim/IsaacLab.git && \
    conda run -n go2_omniverse ./IsaacLab/isaaclab.sh --install

ARG CACHE_BUST=1
#RUN echo "$CACHE_BUST" # Prevent caching by uncommenting this line
# Copy the go2_omniverse directory from the local machine
COPY . /go2_omniverse

RUN echo '{ \
    "file_format_version" : "1.0.1", \
    "ICD": { \
        "library_path": "libGLX_nvidia.so.0", \
        "api_version" : "1.4.303" \
    } \
}' > /usr/share/vulkan/icd.d/nvidia_icd.json

# Build ROS workspaces
RUN pip3 install empy==3.3.4 lark catkin_pkg numpy
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

# Install other dependencies
RUN apt update && apt install -y libxext6 mesa-vulkan-drivers mesa-utils libvulkan1 vulkan-tools libvulkan-dev zenity \
    && rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash", "-c", "source /opt/conda/etc/profile.d/conda.sh && conda activate go2_omniverse && exec bash"]
