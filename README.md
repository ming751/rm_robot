# RM Robot

Welcome to the RM Robot project! This repository contains the controller and trajectory planner of RM robot.

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Introduction
This is a repository for rm bots, you can use it for your bots too.
Mainly includes Cartesian space pose trajectory planning.
Inverse kinematics solution(based ik and optimization)

## Features

- Easy to migrate to other robots
- Optimization method for ik is more robust(based on pinocchio and casadi)
- Trajectory planning is more flexible(Just enter the waypoint)

## Usage

To use this repository, follow these steps:

1. Install the dependencies(we recommend linux platform):

   use conda to install the pinocchio for full version.
   1) update the conda:
    ```bash
    conda update -n base -c defaults conda
    ```
   2) create an conda environment and activate it:
    ```bash
    conda create -n rm_robot python=3.10
    conda activate rm_robot
    ```
   3) add conda-forge channel:
    ```bash
    conda config --add channels conda-forge
    ```
   4) install the pinocchio:
    ```bash
    conda install -c pinocchio
    ```
   5) install the other dependencies:
    ```bash
    pip install -r requirement.txt
    ```
2. Clone the repository:

    ```bash
    git clone https://github.com/ming751/rm_robot.git
    ```
    ```bash
    cd rm_robot
    ```
3. Run the script:

### trajectory planning
    ```bash
    python trajectory_planning.py
    ```
### trajectory optimization
    ```bash
    python controller.py
    ```
### run mujoco simulation
    ```bash
    python mujoco_simulation.py
    ```

## Contributing

We welcome contributions to the RM Robot project! If you would like to contribute, please follow these steps:

1. Fork the repository.
2. Create a new branch for your feature or bugfix:
    ```bash
    git checkout -b my-feature-branch
    ```
3. Make your changes and commit them:
    ```bash
    git commit -am 'Add new feature'
    ```
4. Push your changes to your fork:
    ```bash
    git push origin my-feature-branch
    ```
5. Create a pull request on GitHub.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.
