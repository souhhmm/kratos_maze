# Kratos Maze

```bash
colcon build --packages-select kratos_maze
source install/setup.bash
```

```bash
ros2 launch kratos_maze maze<5x5 or 10x10>.launch.py
ros2 run kratos_maze wall_follower
```