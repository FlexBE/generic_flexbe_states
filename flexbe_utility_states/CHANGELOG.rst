^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flexbe_utility_states
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2023-06-23)
------------------
* pylint testing with colcon test; flag some ROS 2 conversion issues; linter cleanup
* initial release of ROS 2 build; mostly untested other than some basic imports
* Merge pull request `#9 <https://github.com/FlexBE/generic_flexbe_states/issues/9>`_ from alireza-hosseini/install-src
  fix: Install src directories of state packages
* fix: Install src directories of state packages (`#1 <https://github.com/FlexBE/generic_flexbe_states/issues/1>`_)
  This is needed so that the FlexBE App can find the states.
* Merge pull request `#3 <https://github.com/FlexBE/generic_flexbe_states/issues/3>`_ from alireza-hosseini/state_publish_twist
* Implemented publish twist state
  minor cleanup
* Add state export tag
* [flexbe_utility_states] Added state to publish a pose stamped from userdata
* [flexbe_utility_states] Added import tests for logging states
* [flexbe_utility_states] Added package for generic debugging etc states
