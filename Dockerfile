FROM althack/ros2:humble-gazebo-nvidia 

# install utils
ENV DEBIAN_FRONTEND=noninteractive
ENV COM_RCL_EXCEPT=4

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
  gazebo \
  && rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

RUN pip install setuptools==58.2.0

RUN apt update && apt upgrade -y ros-humble-rosidl*

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

WORKDIR /workspaces/drone

RUN git clone -b release/1.14 https://github.com/PX4/PX4-Autopilot.git

RUN cd PX4-Autopilot/ &&  git submodule update --init --recursive

RUN PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx

RUN apt-get update && apt-get install -y gazebo && \
    cd PX4-Autopilot/ && DONT_RUN=1 make px4_sitl gazebo && \
    DONT_RUN=1 make px4_sitl gazebo

RUN mkdir /tmp/px4

COPY ros2_ws ros2_ws
RUN  rm -rf ros2_ws/build/ ros2_ws/log/ ros2_ws/install/
RUN  . /opt/ros/${ROS_DISTRO}/setup.sh && \
     cd ros2_ws && \
     colcon build

COPY ros2_entrypoint.sh ros2_entrypoint.sh

ENV PYTHONOPTIMIZE=1
ENV ROS_DOMAIN_ID=0

RUN echo "source /workspaces/drone/ros2_ws/install/local_setup.bash" >> /home/$USERNAME/.bashrc

RUN chmod +x ros2_entrypoint.sh
ENTRYPOINT ["/workspaces/drone/ros2_entrypoint.sh"]

CMD ["bash"]