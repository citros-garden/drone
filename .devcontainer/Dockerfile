FROM althack/ros2:humble-gazebo-nvidia 

# install utils
ENV DEBIAN_FRONTEND=noninteractive
ENV COM_RCL_EXCEPT=4
ENV PYTHONOPTIMIZE=1
ENV ROS_DOMAIN_ID=0

RUN apt-get update && apt-get install -y \
  bash-completion \
  build-essential \
  cmake \
  gdb \
  git \
  python3-argcomplete \
  python3-pip \
  nano \
  wget \
  curl \
  autoconf \
  automake \
  libtool \
  make \
  g++ \
  unzip \
  sudo \ 
  openssh-server \
  gnupg \
  gdb-multiarch \
  default-jre \
  python3 \
  python3-setuptools \
  python3-vcstool \
  python3-colcon-common-extensions \
  python3-rosdep \
  mesa-utils \
  x11-apps \
  libcanberra-gtk* \
  libglfw3-dev \
  libglew-dev \
  libgl1-mesa-glx \
  libgl1-mesa-dri \
  ros-humble-rosbridge-suite \
  ros-humble-rosbag2-storage-mcap \
  && rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc
RUN pip install xmltodict
RUN usermod -a -G video ros
RUN apt update && apt upgrade -y ros-humble-rosidl*
# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc