FROM althack/ros2:humble-gazebo-nvidia 

# configure environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV COM_RCL_EXCEPT=4
ENV PYTHONOPTIMIZE=1
ENV ROS_DOMAIN_ID=0

# install utils
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
  ros-humble-rosidl* \
  gazebo \
  && rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

RUN pip install setuptools==58.2.0 citros

# define the workdir
WORKDIR /workspaces/drone

# build ros workspace
COPY ros2_ws/src ros2_ws/src
RUN  . /opt/ros/${ROS_DISTRO}/setup.sh && \
     cd ros2_ws && \
     colcon build

# set up px4
COPY PX4-Autopilot PX4-Autopilot
RUN PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
COPY .devcontainer/px4_setup.py .devcontainer/px4_setup.py
RUN python3 /workspaces/drone/.devcontainer/px4_setup.py
RUN apt-get update && apt-get install -y gazebo && \
    cd PX4-Autopilot/ && \
    make clean && \
    DONT_RUN=1 make -j12 px4_sitl gazebo-classic && \
    DONT_RUN=1 make -j12 px4_sitl gazebo-classic

# finial setup
COPY ros2_entrypoint.sh ros2_entrypoint.sh
RUN chmod +x ros2_entrypoint.sh
RUN pip install xmltodict

RUN apt update && apt-get install -y ros-humble-rosbag2-storage-mcap

RUN pip install citros==1.2.28

RUN echo "source /workspaces/drone/ros2_ws/install/local_setup.bash" >> /home/$USERNAME/.bashrc

ENTRYPOINT ["/workspaces/drone/ros2_entrypoint.sh"]

CMD ["bash"]