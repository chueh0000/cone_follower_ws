# Prerequisites & Setup

1. **Environment:** Install ROS 2 (Humble), `rosdep`, `just` (command runner) and `direnv` (env manager).
2. **Authorize Environment:** Run `direnv allow` in the project root to initialize the Python virtual environment and source ROS 2.
3. **Initialize:** Run `just setup` to clone submodules and install all ROS 2 (system) and Python (pip) dependencies.
4. **Simulator:** Run `just download-fsds` to fetch the FSDS binary.
5. **Build:** Run `just build` to compile the workspace.


### Environment Tools Installation Instruction

Before setting up the workspace, ensure you have the following tools installed:

#### 1. Just (Command Runner)
`just` is used to automate builds, simulation, and deployment tasks.
- **Ubuntu/Debian:** 
  ```bash
  sudo apt install just
  ```
- **Pre-compiled Binary (Recommended):**
  ```bash
  curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to /usr/local/bin
  ```

#### 2. Direnv (Environment Management)
`direnv` automatically sources the ROS 2 environment and workspace whenever you enter the project directory.
- **Install:**
  ```bash
  sudo apt install direnv
  ```
- **Shell Hook:** Add the following to your `~/.bashrc` (or `~/.zshrc`):
  ```bash
  eval "$(direnv hook bash)"
  ```
- **Authorize:** Once installed, run `direnv allow` in the project root to enable automatic sourcing.

#### 3. ROS 2 & Rosdep
This project is built using ROS 2 Humble Hawksbill.
- **Install ROS 2:** Follow the [official ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html).
- **Initialize Rosdep:** `rosdep` is used to manage system dependencies.
  ```bash
  sudo apt install python3-rosdep
  sudo rosdep init
  rosdep update
  ```
