```
roslaunch bacchus_gazebo vineyard_demo.launch world_name:=vineyard_small
roslaunch uol_cmp9767m_tutorial topo_nav.launch
rosrun topological_utils load_yaml_map.py $(rospack find grape_counter)/maps/test_mod2.yaml -f
rviz -d uol_cmp9767m_tutorial/config/topo_nav.rviz
rostopic pub /update_map std_msgs/Time "data:
  secs: 0
  nsecs: 0" 

```