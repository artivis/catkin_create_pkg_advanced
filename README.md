catkin_create_pkg_advanced
===

Wrapper over catkin_create_pkg that also adds:


* Add launch folder and example file
* Add config folder and example file
* Add cfg folder and example file
* Add src/<pkg_name> folder and init and example
* Add scripts folder and example file
* Add setup.py also adding requires
* Add rviz folder and empty example
* Add C++ example files

* Modify CMakeLists.txt to:
*  - Add dynamic reconfigure
*  - Add python executable
*  - Add python installation
*  - Add installation of launch and config and rviz
*  - Add c++ example compilation lines

To ease life on creating a new package.

Cpp Example :

    rosrun catkin_create_pkg_advanced catkin_create_pkg_advanced [usual catkin_create_pkg args] my_test_pkg roscpp

After compilation (catkin_make)

    rosrun my_test_pkg my_test_pkg_node

in another terminal

    rostopic pub /my_test_pkg_node/topic_in std_msgs/Empty "{}"

or

    roslaunch my_test_pkg my_test_pkg_roscpp.launch

in another terminal

    rostopic pub /my_test_pkg_node/another_topic std_msgs/Empty "{}"

Voila !
