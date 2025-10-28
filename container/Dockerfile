FROM ros:humble-ros-core
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y \
  python3-bloom \
  python3-rosdep \
  fakeroot \
  debhelper \
  dh-python \
  ros-humble-rmw-zenoh-cpp
RUN rosdep init && rosdep update

WORKDIR /work

COPY container/build_deb_container.sh /usr/local/bin/build.sh
RUN chmod +x /usr/local/bin/build.sh
