This project is a ROS2 workspace that extends the tirrex_workspace.
Its aim is to provide a simple way to work with tirrex and TSCF developments without having to
recompile everytime the packages of the tirrex workspace (there are more than 150 of them).
To do that, it exploits the concept of
[underlay and overlay](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-the-overlay)
defined by ROS.
In this case, we use the following workspaces:
* /opt/ros/humble (underlay)
* tirrex_workspace (underlay)
* this workspace (overlay)

It significates that if you want to modify a package that is defined in tirrex_workspace, instead of
modifying it in tirrex_workspace, you have to clone this package into your workspace and make your
changes locally.
This allows keeping a clean tirrex_workspace and simplify its update.

This workspace is a fork of [template_ws](https://github.com/Tirrex-Roboterrium/template_ws).
It contains drivers to control a panther robot using the control command of INRAE TSCF's team.

# Installation

By default, this workspace is based on the docker image
[tirrex_workspace:full](https://github.com/Tirrex-Roboterrium/tirrex_workspace/pkgs/container/tirrex_workspace).
If docker is not installed on your machine, you first need to follow the instructions of the
[docker official website](https://docs.docker.com/engine/install/)
After that, you can check that you have a docker compose version greater that `2.20` by executing:
```
docker compose version
```

Clone this workspace somewhere on your machine.
From the root of the workspace, execute the script `create_ws` to create a `.env` that contains
some environment variables useful to build the docker images, and also clone demos examples.
```bash
cd <name_of_your_project>
./scripts/create_ws
```

This script will try to find the directory `tirrex_workspace` or `tirrex_ws` at the same level of
this one.
If this workspace is detected, you can choose to use this one instead of the embedded one, but it is
not recommended.

Now you can compile the workspace
```bash
docker compose run --rm compile
```

The first time, this command will create a docker image with the name of your workspace.
This image is based on the one of tirrex_workspace.
Regarding the compilation, it will do nothing because you don't have any packages at the moment.
You can now create your own ROS package in `src` directory and start working!

You can open a shell inside the ROS environment using
```bash
docker compose run --rm bash
```
This command starts an interactive docker container in your workspace.
Everything is already sourced, so you can execute any `ros2 run` or `ros2 launch` with a package of
your workspace or tirrex_workspace.


## Using local tirrex_workspace

This approach is not recommended, but if you want to use a local copy of tirrex_workspace instead of
the one that is included in the docker image, you can follow these instructions.

You first need to install tirrex_workspace by following instructions in the
[README of this workspace](https://github.com/Tirrex-Roboterrium/tirrex_workspace).
If you are an INRAE developer, you have to follow the specific instructions (there is a section
after installation) and use the repository from the INRAE forge.
This workspace must be installed outside of this project.
The best is to install it at the same level of this workspace, because it will be detected by the
`create_env` script and automatically and `TIRREX_WORKSPACE` in the `.env` file at the root of this
project.

If tirrex_workspace is not found, you can manually define where it is using the following command
after replacing the value by the correct path
```bash
echo >>.env TIRREX_WORKSPACE="<path/to/tirrex/workspace>"
```

After that, you can easily switch between the embedded tirrex_workspace and the local one by
changing the value of `TIRREX_IMAGE_TAG` in the `.env` file.
The possible values are:
* `full`: use the tirrex_workspace that is included in the docker image
* `devel`: bind a local version of tirrex_workspace inside the docker container


# Adding libraries or programs in the docker image

If you want to install programs using `apt install`, you can add them in the
[Dockerfile](Dockerfile).
For example, if you want to install the c/c++ debugger `gdb`, you have to edit the Dockerfile this
way:
```Dockerfile
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      gdb \
    && rm -rf /var/lib/apt/lists/*
```

After that, you have to rebuild the docker image
```bash
docker compose build compile
```

If you want to install missing libraries from your ROS packages, the best way to do that is to
correctly add them in the `package.xml` file of your package and rebuild from scratch the docker
image.
```bash
docker compile build --no-cache compile
```
These libraries will be automatically installed using `rosdep`.
If you want more information about managing these dependencies, you can read the
[rosdep tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html).


# Recommended practices with git

You have to create a different git project for every ROS package (or group of dependent packages)
and handle them using [`vcstool`](https://github.com/dirk-thomas/vcstool).
This is the approach used by tirrex_workspace.
You have to define a `docker/repositories` file that contain the URL of each git project you want to
include in your workspace.
If you want to write it manually, you can take inspiration from the
[repositories](https://github.com/Tirrex-Roboterrium/tirrex_workspace/blob/main/docker/repositories)
file from tirrex workspace, but the easiest way is to generate it from the existing git projects you
already have included in your workspace.
You can generate it using
```bash
./scripts/generate_repositories
```
This is an important step to share your work with others because they can automatically clone all
your project using the same script than tirrex_workspace:
```bash
./scripts/create_ws
```

It is also possible to define several `repositories` file if you want to provide different URLs for
the git projects.
It can be useful if you want to provide URLs with a private token that allows downloading the source
code without an account on the git server.
In this case, you have to specify the path of the repositories you want to use before executing
`create_ws`.
This can be done using the following command:
```bash
echo REPOS_FILE="<path/of/your/repositories/file>" >> .env
```
