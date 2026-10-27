<p align="center">
  <img alt="Aero Hand Open by TetherIA" src="https://github.com/TetherIA/aero-hand-open/tree/main/sdk/assets/logo.png" width="30%">
  <br/><br/>
</p>
 
<div align="center">

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![TetherIA](https://img.shields.io/badge/Developed%20by-TetherIA.ai-0A66C2)](https://tetheria.ai)

</div>

<h2 align="center">
  <p>Aero Hand ROS2 — ROS2 package for for TetherIA's Robotic Hand</p>
</h2>

<div align="center">
  <img src="https://github.com/TetherIA/aero-hand-open/tree/main/sdk/assets/banner.jpg" alt="Aero Hand Demo" title="Aero Hand in action" width="70%"/>
  <p><strong>Aero Hand Open</strong> is a 7-DoF tendon-driven robotic hand for dexterous manipulation and research.</p>
</div>

---

## ⚙️ Installation

We currently have tested the ROS2 package on **Ubuntu 22.04** with **ROS2 Humble Hawksbill**.

### Prerequisites

Before installing this package, ensure you have:
- ROS2 Humble installed. Follow the official guide [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
- `colcon` build tool installed. You can install it via:
  ```bash
  sudo apt install python3-colcon-common-extensions
  ```
- The [Aero Hand SDK](https://github.com/TetherIA/aero-hand-open/tree/main/sdk) installed, follow the installation instructions in its README.

### Installation Steps

To install the `aero-open-ros2` package, follow these steps:

1. Clone the repository to your local machine:
   ```bash
   git clone https://github.com/TetherIA/aero-open-ros2.git
   ```

2. Navigate to the cloned repository directory:
   ```bash
   cd aero-open-ros2
   ```

3. Build the packages with colcon:
   ```bash
   colcon build
   ```

4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

---

## 🚀 Next Steps

Take a look at the [Aero Hand Open README](src/aero_hand_open/README.md) next to get started with using the Aero Hand Open ROS2 package.

## 🧰 Troubleshooting

Coming soon — common setup and runtime issues will be documented here.

## 💬 Support

If you encounter issues or have feature requests:
- Open a [GitHub Issue](https://github.com/TetherIA/aero-open-ros2/issues)
- Contact us at **support@tetheria.ai**

---

## 🤝 Contribution

We welcome community contributions!

If you'd like to improve the SDK, fix bugs, or add new features:

1. Fork this repository.
2. Create a new branch for your changes.
    ```bash
    git checkout -b feature/your-feature-name
    ```

3. Commit your changes with clear messages.

4. Push your branch to your fork.

5. Open a Pull Request (PR) describing your updates.


---

## ⚖️ License

This project is licensed under the **Apache License 2.0**.


<div align="center">
If you find this project useful, please give it a star! ⭐

Built with ❤️ by TetherIA.ai
</div>