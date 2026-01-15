# Dingo MuJoCo + ROS 2 (CHAMP-like gait controller)

A minimal ROS 2 workspace for simulating a Dingo quadruped in **MuJoCo** and controlling it with a **C++ CHAMP-like gait + IK controller**.

- Simulator node: `dingo_mujoco` (Python, publishes `/clock`, `/joint_states`, `/odom` + TF)
- Controller node: `dingo_gait_controller_cpp` (C++, subscribes `/cmd_vel`, publishes joint targets)
- Launch/config: `dingo_config` (launch + YAML params + URDF for TF)

---

## English

### 1) Requirements

Recommended setup for Windows users:
- **Windows 10/11 + WSL2 (Ubuntu 22.04)**
- **ROS 2 Humble** installed inside WSL Ubuntu

Inside Ubuntu/WSL you need:
- `python3`
- `pip`
- MuJoCo Python package (`mujoco`) and its runtime deps

Install dependencies (Ubuntu/WSL):

```bash
sudo apt update
sudo apt install -y \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-teleop-twist-keyboard

# MuJoCo python
pip3 install --user mujoco
```

Note:
- If you want the MuJoCo viewer window (`render:=true`) in WSL, you also need a working GUI setup (WSLg on Win11 is easiest).

### 2) Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
cd ~/dingo_ws
rosdep update
rosdep install --from-paths src -i -y --rosdistro humble

colcon build --symlink-install
```

### 3) Run simulation

```bash
./start_simulation.sh
```

Or directly:

```bash
source /opt/ros/humble/setup.bash
source ~/dingo_ws/install/setup.bash
ros2 launch dingo_config mujoco.launch.py render:=true
```

### 4) Keyboard control (recommended: run teleop inside WSL)

Open a second terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/dingo_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

Keys (teleop_twist_keyboard prints help in the terminal):
- `i`, `,` : forward / backward
- `j`, `l` : yaw left / yaw right
- `k` : stop

### 5) Keyboard control from native Windows (optional)

If you prefer running teleop on **Windows (native)** and keep the simulator in **WSL**:

1) Install ROS 2 on Windows (Humble). Microsoft docs provide Windows installers.
2) Ensure DDS can communicate across Windows <-> WSL.

Minimum environment tips:
- Use the same `ROS_DOMAIN_ID` on both sides.
- Disable localhost-only mode:
  - On Windows PowerShell: `setx ROS_LOCALHOST_ONLY 0`
  - On WSL: `export ROS_LOCALHOST_ONLY=0`

Then on Windows, run teleop:

```powershell
# after setting up the ROS 2 environment on Windows
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

If discovery doesn’t work:
- Try setting an explicit RMW implementation consistently on both sides (e.g. CycloneDDS).
- Ensure firewall allows DDS traffic.

### 6) Useful commands

Check topics:

```bash
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /clock
```

Disable the gait controller (sim only):

```bash
ros2 launch dingo_config mujoco.launch.py enable_gait_controller:=false
```

---

## Tiếng Việt

### 1) Yêu cầu

Khuyến nghị cho người dùng Windows:
- **Windows 10/11 + WSL2 (Ubuntu 22.04)**
- Cài **ROS 2 Humble** trong Ubuntu/WSL

Trong Ubuntu/WSL cần:
- `python3`, `pip`
- Gói MuJoCo Python: `mujoco`

Cài dependencies (Ubuntu/WSL):

```bash
sudo apt update
sudo apt install -y \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-teleop-twist-keyboard

pip3 install --user mujoco
```

Ghi chú:
- Nếu muốn mở cửa sổ viewer MuJoCo (`render:=true`) trong WSL thì cần GUI (Win11 + WSLg dễ nhất).

### 2) Build workspace

```bash
source /opt/ros/humble/setup.bash
cd ~/dingo_ws
rosdep update
rosdep install --from-paths src -i -y --rosdistro humble

colcon build --symlink-install
```

### 3) Chạy mô phỏng

```bash
./start_simulation.sh
```

Hoặc chạy launch trực tiếp:

```bash
source /opt/ros/humble/setup.bash
source ~/dingo_ws/install/setup.bash
ros2 launch dingo_config mujoco.launch.py render:=true
```

### 4) Điều khiển bằng bàn phím (khuyến nghị: chạy teleop trong WSL)

Mở terminal thứ 2:

```bash
source /opt/ros/humble/setup.bash
source ~/dingo_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

Phím điều khiển (teleop sẽ in hướng dẫn trong terminal):
- `i`, `,`: tiến / lùi
- `j`, `l`: quay trái / quay phải
- `k`: dừng

### 5) Điều khiển từ Windows native (tuỳ chọn)

Nếu bạn muốn chạy teleop trên **Windows** (native) và chạy mô phỏng trong **WSL**:

1) Cài ROS 2 Humble trên Windows.
2) Cấu hình DDS để Windows <-> WSL nhìn thấy nhau.

Gợi ý tối thiểu:
- Đặt cùng `ROS_DOMAIN_ID` ở cả Windows và WSL.
- Tắt chế độ localhost-only:
  - Windows PowerShell: `setx ROS_LOCALHOST_ONLY 0`
  - WSL: `export ROS_LOCALHOST_ONLY=0`

Sau đó trên Windows chạy:

```powershell
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

Nếu không discover được:
- Thử thống nhất `RMW_IMPLEMENTATION` (ví dụ CycloneDDS).
- Kiểm tra firewall.

---
