^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flexbe_manipulation_states
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2023-06-23)
------------------
* pylint testing with colcon test; flag some ROS 2 conversion issues; linter cleanup
* remove non-standard packages from rosdepend; update .github CI
* initial release of ROS 2 build; mostly untested other than some basic imports
* Merge pull request `#9 <https://github.com/FlexBE/generic_flexbe_states/issues/9>`_ from alireza-hosseini/install-src
  fix: Install src directories of state packages
* fix: Install src directories of state packages (`#1 <https://github.com/FlexBE/generic_flexbe_states/issues/1>`_)
  This is needed so that the FlexBE App can find the states.
* Fix `#8 <https://github.com/FlexBE/generic_flexbe_states/issues/8>`_: Use correct Logger.logerr call
* Added weights and time planning constraints for joint_state_to_moveit state
* Added weight and time planning constraints to srdf_state_to_moveit state
* Minor changes, added documentation for input keys
* Added new joint_state_to_moveit state to send joint_values as input keys
* Added output key values to track desired motion when recovering
* Add state export tag
* Testing stamp = 0 for trajectory commands
* Increased time stamp for trajectory
* Added generic states to dynamicaly set joint values from joiunt names
* Made move_group name input key optional and setted default to empty string
* Removed debug message in execute trayectory state
* Changed tabs for spaces
* [flexbe_manipulation_states] Added wait_for_execution parameter to srdf_state_to_moveit_execute_trajectory and optimized parsing code
* [flexbe_manipulation_states] Created generic state to execute a known trajectory without planning (`#2 <https://github.com/FlexBE/generic_flexbe_states/issues/2>`_)
* [flexbe_manipulation_states] Generic SRDF State to MoveIt (`#1 <https://github.com/FlexBE/generic_flexbe_states/issues/1>`_)
  * Initial state
  * Created a generic state to move a joint group to a state defined in SRDF using MoveIt
* [flexbe_manipulation_states] Added pkg for manipulation and trajectory planning states
