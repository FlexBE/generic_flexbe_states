import os
import sys
import ament_index_python
import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions
import launch_testing_ros
import pytest

@pytest.mark.rostest
def generate_test_description():
    path_to_test = os.path.dirname(__file__)
    flexbe_testing_dir = get_package_share_directory('flexbe_testing')

    pkg = DeclareLaunchArgument(
        "pkg",
        default_value="flexbe_states")
    path = DeclareLaunchArgument(
        "path",
        default_value=path_to_test)

    flexbe_testing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(flexbe_testing_dir + "/launch/flexbe_testing.launch"),
        launch_arguments={
            'compact_format': "False",
            'package': LaunchConfiguration("pkg"),
            "testcases": LaunchConfiguration("path") + "/publish_pose_import.test\n" +
                         LaunchConfiguration("path") + "/publish_twist_import.test\n" +
                         LaunchConfiguration("path") + "/start_record_logs_state_import.test\n" +
                         LaunchConfiguration("path") + "/stop_record_logs_state_import.test\n" 
        }.items()
    )

    return (
        launch.LaunchDescription([
            pkg,
            path,
            flexbe_testing
        ]),
        {
            'flexbe_testing': flexbe_testing,
        }
    )
