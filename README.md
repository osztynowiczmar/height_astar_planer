# Height A* Planner

A ROS 2 Jazzy project implementing the A* path planning algorithm on height maps using the Grid Map framework.

The planner extends the standard A* algorithm by incorporating terrain elevation into the traversal cost. This allows the generated paths to avoid steep slopes and prefer smoother routes while still finding an efficient path to the goal.

---

## Features

- A* path planning on a height map
- Elevation-aware traversal cost
- Maximum slope constraint
- Interactive start and goal selection in RViz
- Visualization of:
  - generated terrain
  - explored cells
  - planned path
- Multiple custom terrain generators
- Planning performance statistics

---

## Project Architecture

The project consists of three ROS 2 nodes:

### `multi_hills_craters_map_publisher_node`

Publishes a terrain containing multiple hills and craters generated using Gaussian functions.

### `labyrinth_map_publisher_node`

Publishes a height map with additional maze-like structures that act as obstacles due to large elevation differences.

### `astar_planner_node`

Main planning node responsible for:

- receiving the height map,
- receiving start and goal positions,
- running the A* algorithm,
- publishing the planned path,
- visualizing explored nodes.

---

## Height Maps

### Multi Hills & Craters

A smooth terrain generated from multiple Gaussian hills and craters.

Features:

- positive elevation peaks,
- negative elevation depressions,
- continuous and smooth surface.

### Labyrinth Map

A more challenging environment containing:

- hills,
- craters,
- maze walls.

Large elevation changes create regions that are expensive or impossible to traverse, forcing the planner to find feasible corridors.

---

## Cost Function

Unlike the standard A* algorithm, the traversal cost depends on terrain slope.

For neighboring cells:

```text
distance = horizontal distance
dz       = z_b - z_a
slope    = abs(dz) / distance
```

The traversal cost is computed as:

```text
cost = distance * (1.0 + SLOPE_WEIGHT * slope)
```

If:

```text
slope > MAX_SLOPE
```

the transition is rejected.

This makes steep terrain significantly more expensive and encourages smoother paths.

---

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- grid_map
- RViz2
- colcon

---

## Build

Clone the repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/osztynowiczmar/height_astar_planer.git
```

Build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select height_astar_planer
source install/setup.bash
```

---

## Running the Project

### Start terrain publisher

Multi-hills terrain:

```bash
ros2 run height_astar_planer multi_hills_craters_map_publisher_node
```

or labyrinth terrain:

```bash
ros2 run height_astar_planer labyrinth_map_publisher_node
```

### Start planner

Open a new terminal:

```bash
source ~/ros2_ws/install/setup.bash

ros2 run height_astar_planer astar_planner_node
```

### Launch RViz

Open another terminal:

```bash
rviz2
```

Add displays for:

- GridMap
- Path
- MarkerArray

and subscribe to the appropriate planner topics.

---

## Selecting Start and Goal

### Start Position

Use RViz tool:

```text
2D Pose Estimate
```

which publishes to:

```text
/initialpose
```

### Goal Position

Use RViz tool:

```text
2D Goal Pose
```

which publishes to:

```text
/goal_pose
```

The planner automatically starts after receiving:

- the map,
- the start pose,
- the goal pose.

---

## Example Performance

### Labyrinth Map

| Metric | Typical Value |
|----------|----------|
| Planning Time | 10–32 ms |
| Visited Cells | 2700–6000 |
| Path Length | 179–284 cells |

### Multi Hills & Craters Map

| Metric | Typical Value |
|----------|----------|
| Planning Time | 10–24 ms |
| Visited Cells | 1700–6400 |
| Path Length | 42–123 cells |

---

## Technologies

- ROS 2 Jazzy
- C++
- Grid Map
- RViz2
- A* Search Algorithm

---

## Authors

Marcin Osztynowicz  
Sebastian Nachowiak

Faculty of Automatic Control, Robotics and Electrical Engineering  
Poznań University of Technology
