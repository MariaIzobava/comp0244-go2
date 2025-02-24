# Basic instructions

### Build and run a node

```bash
colcon build --packages-select cw1_team_9
source install/setup.bash
ros2 run cw1_team_9 bug0
```

### Build and launch

```bash
colcon build --packages-select cw1_team_9
source install/setup.bash
ros2 launch cw1_team_9 run_solution_task_3.launch.py
```
