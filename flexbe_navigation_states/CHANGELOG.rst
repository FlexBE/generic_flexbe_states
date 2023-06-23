^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flexbe_navigation_states
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* pylint testing with colcon test; flag some ROS 2 conversion issues; linter cleanup
* remove non-standard packages from rosdepend; update .github CI
* initial release of ROS 2 build; mostly untested other than some basic imports
* Merge pull request `#9 <https://github.com/FlexBE/generic_flexbe_states/issues/9>`_ from alireza-hosseini/install-src
  fix: Install src directories of state packages
* fix: Install src directories of state packages (`#1 <https://github.com/FlexBE/generic_flexbe_states/issues/1>`_)
  This is needed so that the FlexBE App can find the states.
* Merge pull request `#5 <https://github.com/FlexBE/generic_flexbe_states/issues/5>`_ from alireza-hosseini/fix-move-base-state
  Implemented on_stop method
* Implemented on_stop method
  When using this state in an embedded behavior, it is necessary to implement the "on_stop" method in order to preempt the move base goal.
  Added cancel_active_goals method
  Fixed major bug by adding a condition to check if the action client is already available and active.
* Add state export tag
* [flexbe_navigation_states] Added import test for move base state
* [flexbe_navigation_states] Changed MoveBaseState to accept a Pose2D waypoint as input key
* [navigation] Fix setup.py
* [navigation] Add on_exit() to move_base state
* First commit:
  * Add Python .gitignore
  * Add README
  * Add move_base_state
