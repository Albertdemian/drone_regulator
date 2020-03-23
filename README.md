# drone_regulator

# To launch all nodes: 

**to launch dependencies and topics:**

  >roslaunch drone_sim forest_sim.launch 
  
**to launch ground control:** 
  
  >roslaunch ground_control ground_control.launch 
  
**to launch configuration files and gui:**

  >rosrun dynamic_tutorials server.py
  >rosrun dynamic_tutorials tune_server.py
  >rosrun rqt_gui rqt_gui -s reconfigure 
  
**to launch repulsive field node:**

  >rosrun dynamic_tutorials field.py 
  
**to launch offboard mode:**

  >rosrun dynamic_tutorials offb_node.py

Next assignment I will create a launch file for all these nodes
