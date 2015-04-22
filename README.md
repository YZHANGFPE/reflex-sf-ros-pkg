# reflex-sf-ros-pkg
Collection of ROS packages for the ReFlex SF hand (Modified)

Tested on Ubuntu-14.04 with ROS indigo  
Hardware mounting http://www.righthandrobotics.com/main:reflex-sf  
Power adapter: 12V 5A   http://www.amazon.com/gp/product/B008FKDK2M/ref=od_aui_detailpages00?ie=UTF8&psc=1

## Installation

1. Install the ROS dynamixel-controllers package  

  `sudo apt-get install ros-indigo-dynamixel-controllers`

2. Clone the reflex-sf-ros-pkg  

   `cd ~/catkin_ws/src`  
   `git clone https://github.com/YZHANGFPE/reflex-sf-ros-pkg.git`

3. Make the package  

   `cd ~/catkin_ws`  
   `catkin_make`
   
## Operation

1. Launch the motor controller  

  `roslaunch reflex_sf reflex_sf.launch`
  
2. Calibration
   
   The original calibration.py did not provide sufficient capability.
   My approach was to modify the configuration file of the motor controller reflex_sf.yaml.
   The item I changed was the value of motor: init: for each motor.  
   **Note: If it was applied to a new hand, the values should be reset to zero. 
   The values in the current file is only for the hand in our lab.**
   
3. Test

   `rosrun reflex_sf test.py`  
   The hand will perform a cylindrical grasp and a pinch grasp.  
   Video demo:  http://youtu.be/EFVr8Z6VKyc 
