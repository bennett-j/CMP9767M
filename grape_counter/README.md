# Grape Counter Package
A Thorvald robot autonomously navigates a simulated vineyard and counts the number of grape bunches. A topological map corresponding to the vineyard is provided with nodes at either end of each row. The `navigator.py` file contains the mission and sends the robot to the topological nodes in order and instructs the other ROS nodes their behaviour on each leg. Whilst counting, the robot comes to a stop before capturing an image to process. It identifies the bunches and their location and counts it if it has not already been identified.

## Preparation
Install the package and ensure its dependencies are installed. Notably it depends on some functionality from `uol_cmp9767m_tutorial` and `tf2`. These are specified in `package.xml`. Build using the catkin build system to ensure the custom service messages are built.

## Running
1. Start the simulation and other nodes using `roslaunch grape_counter grape_counter.launch`.
1. Run `rosrun topological_utils load_yaml_map.py $(rospack find grape_counter)/maps/vineyard_small_map.yaml` to load the opological map. The `-f` may be required to overwrite an existing map.
1. Commence the task by running the navigator node which contains the mission using `rosrun grape_counter navigator.py`. This node will report progress and final result to the screen.