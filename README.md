# Moving_turtlebot3
this Rebo contains files and the modifications you have to do in setup.py and package.xml files to make your turtlebot3 moves and we used service methode in communication between the nodes 
##modification in setup.py file :
##under the console_scripts you have to make your nodes excutable 
  ####  "server=moveturtlebot3.server_turtlebot3:main",
  ####  "client=moveturtlebot3.client_turtlebot3:main",
     
##Then, in package.xml you have to add the dependencies :
 #### <exec_depend>rclpy</exec_depend>
 #### <exec_depend>std_msgs</exec_depend>
 #### <exec_depend>geometry_msgs</exec_depend> 
