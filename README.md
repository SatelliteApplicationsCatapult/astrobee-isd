# astrobee-isd
In-station demonstrator project for the Astrobee robot aboard the ISS

Catalytic project name:

**BEECORE - Behavioural Embeddings for Exploration, Capture, Object Recognition & Execution**

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
export ANDROID_PATH=$ASTROBEE_WS/src/submodules/android
export ANDROID_HOME=$HOME/Android/Sdk
export EMULATOR=$HOME/Android/Sdk/emulator/emulator
export AVD=Nexus_5_API_25
export USER=root
export CUSTOM_WS=/src/astrobee-isd
export GAZEBO_MODEL_PATH=$CUSTOM_WS/models

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

<br>


# How to run the training environment - Clean start from reboot

## Terminal Setup
```bash
# Initialise container
XAUTH=/tmp/.docker.xauth && touch $XAUTH && xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
xhost +local:docker
docker start astrobee

# Terminal 1 - Emulator
docker exec -it astrobee bash
cd $ANDROID_PATH/scripts
./launch_emulator.sh

# Terminal 2 - Simulation
docker exec -it astrobee bash
roslaunch astrobee_ros_demo astrobee_joy_test_sim.launch

# Terminal 3 - GUI
docker exec -it astrobee bash
cd $CUSTOM_WS/ros_ws_astrobee/src/beecore_capture_gui/
python3 run_gui.py
```

## GUI Preparation
- Wait for the simulation to initialise
    - Some warning/errors in the CLI about heartbeat and nodelet can be ignored
    - Confirm that in RViz the pointcloud data is displaying correctly
- Open browser windows
    - http://localhost:8090/
    - http://localhost:8090/view
- Check that gamepad (joystick) USB receiver is plugged in
- Press Start on gamepad to wake it up
- In BEECORE Capture GUI:
    - Switch to 'Experiment Control' tab
    - Enable 'Control Services' in order:
        - Joystick Node
        - Joystick Command Converter
        - Custom FAM Control
    - Using the gamepad and observing the RViz window, visually confirm that the force/torque
      commands (green/purple arrows) are coming through, but the robot does not yet move
    - Press the 'Clear Astrobee Fault State' button, so that the flag 'ff_msgs/FaultState' reads
      '0 = STARTING_UP'
    - Now toggle the activation call to the control node:
        -  Custom FAM Control Start
    - At this stage, all feedback LEDs on the right-side panel of the GUI should be green
    - Confirm the robot now moves in response to gamepad inputs
- Now the system is ready for multiple experimental runs to take place

## Experiment Run Preparation
- Set the desired tool initial conditions under the 'Tool Spawn' section
- Set rosbag suffix to a desired name (the ID and timestamp will increment automatically, to ensure
  folder name uniqueness)
- Under the 'Camera Control' tab, the four camera views can be adjusted if necessary, but it is
  recommended they are kept as default
- Under the 'Topics' tab, any currently available topics can be selected for recording
- Under the 'Other' tab, other settings can be adjusted, but again it is recommended they are kept
  as default
- Repeat these steps as required

## Recording a Run
- On the 'Experiment Control' tab, press the 'Reset Experiment' button
    - This repositions the robot and spawns the selected tool with set initial conditions
- Press 'Start Recording'
- Once the experimental run has been completed, manually determine if the capture was a success or
  failure. An option must be set before the 'Stop Recording' button can be pressed
- Press 'Stop Recording' to end the recording
- The rosbag will now be saved under the stated folder
- Repeat these steps as required

## Notes
- Sometimes the pointcloud (PerchCam) sensor does not initialise correctly, and only displays a
  small spherical pointcloud at origin. If this happens, restart the simulation.
- RViz uses a dummy fixed frame 'rviz_up' to display the Astrobee body upright, i.e. with perch arm
  on top. The Gazebo simulation uses Z axis up, which is why the ISS looks upside-down. This is not
  easily changeable without making changes to the world/models, so is left as default.
- It is recommended to leave 'Experiment ID' on the current value, as this will auto-increment on
  each recording. However it can be reset at any time if needed, e.g. due to a bad recording that
  does not need to be kept and gets manually deleted from disk.
- If the joystick has not appeared in the container (/dev/input/js0 does not exist), then instead
  of restarting the container, you can do the following:
  ```bash
  mknod /dev/input/js0 c 13 0
  chmod 666 /dev/input/js0
  ```
