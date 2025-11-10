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
    rosdep install -iyr --from-paths /tmp/src && \
    rm -rf /var/lib/apt/lists/*

# you can add here ubuntu packages that you want to install (or uncomment the existing ones)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3-pyproj \
      # gdbserver \
      # valgrind \
      # strace \
    && rm -rf /var/lib/apt/lists/*
