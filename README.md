# ROS 2 "Bump and Go" Demo

![TurtleBot3 in RViz](https://www.robotis.us/images/product/turtlebot3/turtlebot3_friends.png)

This project provides a simple demonstration of a reactive navigation algorithm known as "Bump and Go." It features a simulated TurtleBot3 robot in an office environment, controlled by a ROS 2 node written in Python. The robot uses its LIDAR sensor to detect obstacles and changes its direction when it gets too close.

The primary goal of this repository is to showcase a modern robotics workflow by encapsulating the entire application into a **universal Docker container**. This container can run on any machine without requiring a dedicated NVIDIA GPU, making it highly portable and accessible.

## Features

- **Algorithm:** Simple "Bump and Go" logic.
- **Robot:** TurtleBot3 Burger.
- **Environment:** An office simulation world (`office_small.world`).
- **Framework:** ROS 2 Humble.
- **Simulator:** Gazebo (running in `headless` mode for universal compatibility).
- **Visualization:** RViz2.
- **Deployment Technology:** Docker Container.

---

## Prerequisites

To run this project, you only need a Linux system (Ubuntu 22.04 is recommended) with **Docker** installed.

1.  **Install Docker Engine:**
    Follow the official installation guide for your operating system.
    - [Docker installation instructions for Ubuntu](https://docs.docker.com/engine/install/ubuntu/)

2.  **Manage Docker as a non-root user (Recommended):**
    To avoid using `sudo` for every Docker command, add your user to the `docker` group.
    ```bash
    sudo groupadd docker
    sudo usermod -aG docker $USER
    ```
    After running this, you must **log out and log back in** or **reboot your machine** for the group changes to take effect.

**That's it!** You do not need to install ROS 2, Gazebo, or any other dependencies on your host machine. Everything is self-contained within the Docker container.

---

## 🚀 How to Run the Demo

The process is very straightforward and consists of 3 steps.

### 1. Clone the Repository

Open a terminal and clone this project to your local machine.

```bash
git clone https://github.com/your-username/first-ROS2-project.git
cd first-ROS2-project
