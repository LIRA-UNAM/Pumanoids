
# ⏻ robot_upstart

> Forked from https://github.com/clearpathrobotics/robot_upstart

This package provides an utility to manage ROS2 nodes as systemd services, allowing them to be **automatically started on boot** and easily managed with standard systemd tools.

## ⬇️ Installation

> [!WARNING]
> **Do not** install the APT package, as it is incomplete and unusable by default, following the "*security first*" ROS2 philosophy.

This package is provided as a git submodule at `Pumanoids/colcon_ws/src/extras/`. If the package isn't in the directory, run:
```bash
git submodule init
git submodule update
```
Or clone it manually in the `src/extras/` directory:
```bash
git clone https://github.com/gmsebastian/robot_upstart.git
```

## 💻 Usage

### 🛠️ Create a systemd service for a ROS2 node:

```bash
ros2 run robot_upstart install <package_name>/launch/<launch_file.launch.py> --job <service_name> --setup <path/to/setup.bash> --symlink
```

Where:
- `<service_name>`: Name the systemd service will have.
- `<path/to/setup.bash>`: Specifies the setup.bash file to be used.
- `--symlink`: Creates a symlink that allows the launch file to be modified without needing to reinstall the service.

> [!WARNING]
> The path to the launch file must start with the package directory.
>
> **Absolute paths won't work.**

after that, restart the systemd daemon to apply the changes:

```bash
sudo systemctl daemon-reload
```

### 💡 Manage the service:

```bash
sudo systemctl <action> <service_name>
```

Where `<action>` can be:

- `start`: To start the service.
- `stop`: To stop the service.
- `enable`: To enable the service to start at startup.
- `disable`: To disable the service to start at startup.

#### 🔎 Watch the ROS2 launch logs

```bash
journalctl -u <service_name> -f
```

### ⛔ Remove the service

```bash
ros2 run robot_upstart uninstall <service_name>
```

And restart the systemd daemon:

```bash
sudo systemctl daemon-reload
```

### 📌 Convenient aliases

To avoid long systemd commands, I suggest adding the following bash functions to your `~/.bashrc` to manage the services.

> [!TIP]
> Add these definitions to your `~/.bashrc`.

For `start`, `stop`, `enable` or `disable` services:

```bash
robot() {
    sudo systemctl "$1" "$2"
}
```

For watching system logs:

```bash
robotlog() {
    journalctl -u "$1" -f -o cat
}
```

This way, you can run 
- `robot start <service>` instead of `sudo systemctl start <service>`
- `robotlog <service>` instead of `journalctl -u <service> -f -o cat`
