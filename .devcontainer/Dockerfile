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
  python3-pip \
  python3-vcstools \
  python3-colcon-common-extensions \
  gazebo \
  && rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc



