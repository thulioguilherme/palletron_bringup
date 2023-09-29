# The Palletron bringup repository

This repository contains the files for starting the Palletron.

## Dependencies

* [Navigation2](https://github.com/ros-planning/navigation2)
* [IRA Laser Tools](https://github.com/thulioguilherme/ira_laser_tools)

## Getting Started

* On your workspace, clone this repository.
  ```bash
  git clone https://github.com/thulioguilherme/palletron_bringup.git
  ```

* Install dependencies.
  ```bash
  rosdep install --from-paths src --ignore-src -r -y
  ```

* Build it!
  ```bash
  colcon build
  ```