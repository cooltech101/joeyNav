# Use Ubuntu 22.04 as the base image
FROM ubuntu:22.04

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Set locale
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Setup sources
RUN apt-get update && apt-get install -y software-properties-common curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete

# Initialize rosdep
RUN rosdep init && rosdep update

# Install git and python3-pip
RUN apt-get install -y git python3-pip
RUN pip3 install gdown

# Install Poetry
RUN pip3 install poetry

# Clone the VisualNav-Transformer repository
RUN git clone https://github.com/Robotecai/visualnav-transformer-ros2.git /visualnav-transformer

# Set up environment variables
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Clean up
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /visualnav-transformer

# Install dependencies using Poetry
RUN poetry install --no-interaction --no-ansi

RUN mkdir /visualnav-transformer/model_weights
RUN gdown https://drive.google.com/uc?id=1YJhkkMJAYOiKNyCaelbS_alpUpAJsOUb -O /visualnav-transformer/model_weights/nomad.pth

# Set the entrypoint
ENTRYPOINT ["/bin/bash"]
