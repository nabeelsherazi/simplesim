# simplesim

little sfml + ros2 exercise implementing a pure pursuit trajectory controller

![image](https://github.com/user-attachments/assets/b7e99e1b-69a5-4958-a23f-446d053a0bfc)

## build and run (local)

have ros2 jazzy installed and sourced

```sh
cd ~/src/ros2_ws     # or wherever your colcon workspace is
git clone git@github.com:nabeelsherazi/simplesim src/simplesim
colcon build
```

run

```sh
. install/setup.bash
ros2 run simplesim simplesim_node
```

## build and run (docker)

```sh
xhost +local:docker
docker build -t simplesim:latest .
docker run --rm --device=/dev/dri -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix simplesim:latest
```

## about the wind and drag model

the wind vector is modeled as a 2d vector with randomly walking components.

drag is modeled like this. the real equations are quadratic in velocity, but since the reynolds number also depends on velocity, there's also a linear regime when the reynolds number is decreasing like 1/v. at some point it plateaus though and you enter the quadratic regime, usually around Re=1000. for most objects, this happens at very slow speeds, like less than 0.5 m/s you're in the quadratic regime, but this program models both anyway because using quadratic drag for an object starting from rest looks unphysical.