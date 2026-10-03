# 01 — Environment Setup

## 1. Clone PX4-Autopilot and run the setup script

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

This installs build dependencies and the matching Gazebo version for
your PX4 checkout — do not install Gazebo separately unless the script
fails partway through.

**Reboot or re-log** after this script completes (it modifies group
membership / udev rules in some cases).

## 2. Verify Gazebo installed correctly

```bash
gz sim --version
```

Should report a Gazebo Harmonic (or newer) version string. If `gz` is
not found, check that `~/.bashrc` was updated by the setup script and
re-source it (`source ~/.bashrc`).

## 3. Install ROS 2 (if not already installed)

Follow the official ROS 2 install instructions for your Ubuntu version:
- Ubuntu 22.04 → ROS 2 Humble
- Ubuntu 24.04 → ROS 2 Jazzy

```bash
# example for Humble — adjust distro name if on 24.04
sudo apt install ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 4. Build and install the micro-XRCE-DDS Agent

This is the bridge between PX4's internal uORB topics and ROS 2.

```bash
cd ~
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib/
```

Confirm the checked-out tag matches what your PX4 version expects — if
`main` on PX4-Autopilot has moved to a newer XRCE-DDS protocol version,
check `PX4-Autopilot/msg/px4_ros_com` or the PX4 ROS 2 docs for the
currently recommended Agent version/tag before building.

## 5. Build PX4 once to confirm the toolchain works

```bash
cd ~/PX4-Autopilot
make px4_sitl
```

This should complete without errors and drop you into the PX4 shell
(`pxh>`). Type `quit` (or Ctrl+C) to exit — this step only confirms the
build toolchain; the actual simulation run is in the next file.

## Checklist before moving on

- [ ] `gz sim --version` returns a version
- [ ] `ros2 topic list` runs without error (after `source /opt/ros/<distro>/setup.bash`)
- [ ] `MicroXRCEAgent --help` runs without error
- [ ] `make px4_sitl` builds cleanly from `~/PX4-Autopilot`
