alias sensors="roslaunch sensors sensors.launch" # launches sensors
alias drytest="roslaunch propulsion drytest.launch" # launches drytest
alias zero="rostopic pub /propulsion/microseconds auv_msgs/ThrusterMicroseconds 'microseconds: [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]'" ## zero out thruster microseconds
alias bringup="roslaunch bringup bringup.launch" # launches bringup