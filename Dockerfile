ARG FROM_IMAGE=ghcr.io/tirrex-roboterrium/tirrex_workspace:devel

FROM ${FROM_IMAGE}

# create the same user inside the docker image than the one on your host system
ARG UID GID HOME USER
RUN groupadd -g "${GID}" "${USER}" && \
    useradd -u "${UID}" -g "${GID}" -s /bin/bash -d "${HOME}" -m -G dialout "${USER}"

# install all missing packages that you have specified into your package.xml
RUN --mount=type=bind,source=src,target=/tmp/src \
    apt-get update && \
    rosdep update && \
    rosdep install -iyr --from-paths /tmp/src --skip-keys="gazebo_plugins gazebo_ros gazebo_dev" && \
    rm -rf /var/lib/apt/lists/*

# you can add here ubuntu packages that you want to install (or uncomment the existing ones)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3-pyproj \
      # gdbserver \
      # valgrind \
      # strace \
    && rm -rf /var/lib/apt/lists/*

COPY ./src/ /opt/tirrex_ws/src/third_party/panther/

RUN bash -c "source /opt/ros/humble/setup.bash && \
    colcon --log-base /dev/null build --symlink-install --event-handlers event_log- log- --packages-select \
    panther_description \
    path_following_with_remap \
    panther_bringup \
    source install/setup.bash"