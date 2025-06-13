# Kratos Maze
This is a starter repository for Project Kratos' Maze Competition as part of the 2025 autonomous subsystem induction process.

## Prerequisites
- [Ubuntu 22.04](https://releases.ubuntu.com/jammy/) or equivalent VM
- [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
- [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)

## Installation

### Clone the Package
```bash
cd ros2_ws/src
git clone https://github.com/souhhmm/kratos_maze.git
```

### Build the Package

```bash
colcon build --packages-select kratos_maze
source install/setup.bash
```
To be safe, build the package and source everytime you make a change. Or, `--symlink-install` might be of interest, take a look at the [docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html).

## How to Run
There are two orthogonal simple mazes provided to you with no loops, obstacles etc.

<div style="display: flex; justify-content: center; align-items: center; text-align: center; gap: 10px;">
  <figure style="margin: 0;">
    <img src="assets/maze10x10.svg" style="filter: brightness(0) invert(1); width: 200px;">
    <figcaption><em>10×10 Maze</em></figcaption>
  </figure>
  
  <figure style="margin: 0;">
    <img src="assets/maze5x5.svg" style="filter: brightness(0) invert(1); width: 200px;">
    <figcaption><em>5×5 Maze</em></figcaption>
  </figure>
</div>

### Launch the Maze Environment

```bash
cd ros2_ws/src/kratos_maze
ros2 launch kratos_maze maze5x5.launch.py # for the 5x5 maze
ros2 launch kratos_maze maze10x10.launch.py # for the 10x10 maze
```

You should see the Gazebo environment.
<div style="display: flex; justify-content: center; align-items: center; text-align: center; gap: 10px;">
  <figure style="margin: 0;">
    <img src="assets/gazebo5x5.png" style="width: 500px;">
    <figcaption><em>Gazebo Maze Environment</em></figcaption>
  </figure>
</div>


### Launch a Script
There is a sample wall following algorithm script (based on [this](https://www.youtube.com/watch?v=1l9IMXd33K4&ab_channel=HeyYK)) provided to you. This takes ~17 mins to solve the given 10x10 maze!

Run this in another terminal. You might want to use [terminator](https://gnome-terminator.org/) emulator for multiple terminals in one window.

```bash
ros2 run kratos_maze wall_follower
```

Nice! Now, you should see the bot starting to solve the maze.

To create your own script, add it in the `src/` directory and make sure to update the `setup.py` file with your node.

<details>
<summary> Tip for VS Code users</summary>

To resolve the import errors (yellow squiggles) create a `.env` file in the root of your workspace and add `PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages` in it.
</details>


## What's the competition?

Solve the given 10x10 maze in the fastest time possible!

### Some Rules
1. The bot must navigate from the designated start point to finish point in the shortest time without any external intervention. No tricks allowed, for example, you can't use the outer boundary of the maze xD.

2. The timer starts as soon as you run your script.

3. You would be required to submit your code, along with a video of the bot solving the maze, so make sure you record the process.

<hr></hr>

Join the [Discord](linktodiscord) server for regular updates. Have fun!