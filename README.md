# astrobee-isd
In-station demonstrator project for the Astrobee robot aboard the ISS

Placeholder Catalytic project name:

ASCEND - Astrobee System for Collaborative Engineering & Novel Deployment

<br>


# Prerequisites

## Tutorials
The quick-launch assumes you have followed the instructions below, and set up **both** the Astrobee Simulator and Android Emulator on a new docker container, based on Ubuntu 20.04 (Focal).
- https://nasa.github.io/astrobee/v/develop/md_INSTALL.html
- https://github.com/nasa/astrobee/blob/master/scripts/docker/readme.md
- https://github.com/nasa/astrobee_android/blob/master/guest_science_readme.md
- https://github.com/nasa/astrobee_android/blob/master/emulator.md
- https://github.com/nasa/astrobee_android/blob/master/running_gs_app.md

## Container

Following the instructions and the [run.sh script](https://github.com/nasa/astrobee/blob/master/scripts/docker/run.sh), the container was created as such:

Using --dry-run we can see the dry run results of the script:
```bash
export ASTROBEE_WS=$HOME/code/astrobee
cd $ASTROBEE_WS/src
sudo bash ./scripts/docker/run.sh --remote --focal --gpu=true --dry-run
opts:  --remote --focal --gpu 'true' --dry-run --
Dry run
+ cd /home/dt013/code/astrobee/src/scripts/docker
+ docker run -it --rm --name astrobee --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw --volume=/tmp/.docker.xauth:/tmp/.docker.xauth:rw --env=XAUTHORITY=/tmp/.docker.xauth --env=DISPLAY --privileged --gpus all ghcr.io/nasa/astrobee:latest-ubuntu20.04 /astrobee_init.sh roslaunch astrobee sim.launch dds:=false robot:=sim_pub
```
Notes:
- Use --focal for Ubuntu 20.04
- The default GPU check fails because of a case sensitivity issue, so we set explicitly with --gpu=true param (it then sets --gpus flag)
- Remove --rm flag so the container is not removed on exit
- Add --net=host to be able to use the same network as host
- Remove the last parts as we don't want to auto-run the script or the sim

Edited, this becomes:
```bash
cd $ASTROBEE_WS/src/scripts/docker
docker run -it --net=host --name astrobee --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw --volume=/tmp/.docker.xauth:/tmp/.docker.xauth:rw --env=XAUTHORITY=/tmp/.docker.xauth --env=DISPLAY --privileged --gpus all ghcr.io/nasa/astrobee:latest-ubuntu20.04
```
On host reboot, you will have to recreate the .docker.xauth file, and give the container access to the X server (see instructions below).

## Bashrc

Put useful vars in container's bashrc, and setup ROS:

```bash
# Useful vars
export ASTROBEE_WS=/src/astrobee
export SOURCE_PATH=$ASTROBEE_WS/src
export ANDROID_PATH=$ASTROBEE_WS/src/submodules/android/
export ANDROID_HOME=$HOME/Android/Sdk
export EMULATOR=$HOME/Android/Sdk/emulator/emulator
export AVD=Nexus_5_API_25
export USER=root
export CUSTOM_WS=/src/astrobee-isd/

# ROS
export ROS_IP=$(getent hosts llp | awk '{ print $1 }')
export ROS_MASTER_URI=http://${ROS_IP}:11311
source $ASTROBEE_WS/devel/setup.bash
```

<br>


# How to run the full simulation

```bash
# If container is stopped
# Recreate the temp xauth file:
XAUTH=/tmp/.docker.xauth && touch $XAUTH && xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
# Allow container access to X server:
xhost +local:docker
# Start container:
docker start astrobee

# Terminal 1 - Android Emulator
# Enter container:
docker exec -it astrobee /astrobee_init.sh bash
# Launch the Emulator:
cd $ANDROID_PATH/scripts
./launch_emulator.sh

# Terminal 2 - Astrobee Simulator
# Enter container:
docker exec -it astrobee /astrobee_init.sh bash
# Launch the Simulator:
roslaunch astrobee sim.launch dds:=false robot:=sim_pub rviz:=true

# Terminal 3 - Android Emulator Setup and GDS Simulator
# Enter container:
docker exec -it astrobee /astrobee_init.sh bash
# Wait ~30 seconds after launching the Emulator:
cd $ANDROID_PATH/scripts
./hlp_setup_net.sh -e -h 10.42.0.36 -l 10.42.0.34 -w -10 -t 60
# Make sure hlp ping is stable:
ping hlp
# Restart the Guest Science Manager:
$ANDROID_PATH/scripts/gs_manager.sh restart
# Run the GDS Simulator:
cd $SOURCE_PATH/tools/gds_helper/src
python gds_simulator.py
```
