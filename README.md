
<h1 align="center">
    <a><img src="./Documentation/Images/PUMANOIDS_title.svg" width="500" align="middle" alt="Pumanoids Logo"></a>
</h1>

<p align="center">
    <b>
        Official repository of the Pumas team for the RoboCup Soccer Humanoid League.
    </b>
</p>

> [!IMPORTANT]
> **Rama experimental `test_goalkeeper`.** Esta rama parte de
> `beijing_demo` y contiene el portero configurable para Booster T2: panel web,
> selección Kick/VisualKick, predictor de tiros y bloqueo lateral. El predictor
> está desactivado por defecto y exige localización calibrada al activarse.
>
> - [Inicio rápido y compilación](./beijing/README.md)
> - [Guía completa del portero](./Documentation/Goalkeeper/README.md)
> - [Inventario técnico y criterios de aceptación](./Documentation/Goalkeeper/IMPLEMENTATION.md)

<div align="center">
    <h4>
        <a href="/Documentation/Guides/Installation.md">
            Installation
        </a>
        <span> | </span>
        <a href="/Documentation/Guides/Usage.md">
            Usage
        </a>
        <span> | </span>
        <a href="https://lira.unam.mx/">
            Website
        </a>
    </h4>
</div>
    
<p align="center">
    <img src="./Documentation/Images/PumasTMR2026.jpg" width="500" align="middle" alt="Pumas Team">
</p>

<p align="center">
    <a><img src="https://img.shields.io/github/commit-activity/t/LIRA-UNAM/Pumanoids?style=for-the-badge&color=00254a" alt="Commit Activity"></a>
    <a><img src="https://img.shields.io/github/last-commit/LIRA-UNAM/Pumanoids?style=for-the-badge&color=013b75" alt="Last Commit"></a>
    <a><img src="https://img.shields.io/github/repo-size/LIRA-UNAM/Pumanoids?style=for-the-badge&color=005eb8" alt="Repo Size"></a>
</p>

<p align="center">
    <a href="https://lira.unam.mx/">
        <img src="./Documentation/Images/Developed_at_LIRA.png" height="40" align="middle" alt="Developed at LIRA">
    </a>
</p>

## Getting Started

### Prerequisites

- [Ubuntu 22.04](https://releases.ubuntu.com/jammy/) or later.
- [ROS2 humble](https://docs.ros.org/en/humble/Installation.html) (`ros-humble-desktop` and `ros-dev-tools` required).
- [jetson-utils](https://github.com/dusty-nv/jetson-utils) (only Booster K1).
- [nlohmann-json3-dev](https://github.com/nlohmann/json). Available to install through apt.
- [Docker](https://docs.docker.com/engine/install/ubuntu/).
- A lot of patience.

> ℹ️ **to-do**: add the remaining dependencies

### Installation

First, clone this repository:

```bash
git clone --recurse-submodules https://github.com/LIRA-UNAM/Pumanoids.git
cd Pumanoids
```
> [!NOTE]
> If you didn't clone the repository with the `--recurse-submodules` flag, you can initialize the submodules with the following commands:
> ```bash
> git submodule init
> git submodule update
> ```

Then, follow the instructions in the [Installation Guide](/Documentation/Guides/Installation.md) to set up the **development environment** and install the necessary **dependencies**.

## Building

After cloning the repository and installing the dependencies, build the project at the root of the workspace:

```bash
cd colcon_ws/
colcon build
```

If the build was successful, source the workspace:

```bash
source install/setup.bash
```

## Usage

After initializing the environment, run the following command to launch the main nodes:

```bash
ros2 launch surge_et_ambula state_machine.launch.py robot:=<robot_model>
```

where `<robot_model>` can be:
- `k1`
- `t1`

---

> [!NOTE]
> For further instructions on how to run the code and use the different nodes, please refer to the [Usage Guide](/Documentation/Guides/Usage.md).

## License

Distributed under the **MIT License**. This allows for modification, distribution, and commercial use, provided the original copyright and license notice are included.

See [LICENSE](/LICENSE) for details.

&copy; 2025 - 2026 **Laboratorio de Investigación en Robotica Avanzada**
