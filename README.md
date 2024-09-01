# Description

Created a limo controller using C++. 

If want to see plots for current run, uncomment Plotter node in py_utils/setup.py and in limo.launch.py.

To change parameters for project, change in py_utils/config/params.yaml.

## Approach

The main loop runs as follow: 

1) Turn to destination direction
2) Move to destination
3) Turn to goal direction

Destination direction is calculated via trigonometry. To know if the robot is at destination by querying the incoming odometry data and comparing it. To ensure higher accuracy, slower speed is used to query odometry data at slower rate. 

## Improvements

1) To make the simulation more realistic, I could decelerate the robot at a constant linear rate for both angular and linear velocities instead of change to the slow velocity abruptly.
2) The turning is not limited, would rework function such that the robot turns a maximum of PI radians.
3) I did not have the time to look too much into the limo_simulation, but I would refactor the launch into a global node and move the config files into this global node.

### Software Structure
```
- docker -- Where the Dockerfile lives.
- scripts -- Where necessary external scripts live.
- workspace -- Where all the packages live.
- graphs -- Where all plots for specific turn rates and specific velocities
```

### Graphs

Each folder within graphs describes angular velocity and a linear velocity. Each graph will provide a final delta which is error in its final position. In the end I decided on a slower factor to increase accuracy for longer distances due to the emerging patterns of slower speeds providing lower errors. To account for the slow speeds, I added a threshold in which the slow speed will commence.

For higher accuracies, in params.yaml, decrease d_x_slow and d_theta_slow. For faster run times (but lowered accuracies), increase d_x and d_theta. To try to maintain accuracies, increase threshold_x and threshold_theta.

### Build the simulator

```bash
./scripts/build/sim.sh
```

### Run the simulator

```bash
./scripts/deploy/devel.sh # To enter the docker container
ros2 launch limo_simulation limo.launch.py # To launch the simulator
```

Feel free to modify anything else if it does not work as expected.