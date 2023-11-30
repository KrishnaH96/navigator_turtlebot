# /**
#  * @file navigator_launch.py
#  * @brief Launch file to launch navigator node along with ros bag instructions.
#  *
#  * @author Krishna Rajesh Hundekari
#  * @date 2023-11-29
#  * @copyright 2023 Krishna Rajesh Hundekari
#  * Apache License Version 2.0, January 2004
#  * Licensed to the Apache Software Foundation (ASF) under one
#  * or more contributor license agreements.  See the NOTICE file
#  * distributed with this work for additional information
#  * regarding copyright ownership.  The ASF licenses this file
#  * to you under the Apache License, Version 2.0 (the
#  * "License"); you may not use this file except in compliance
#  * with the License.  You may obtain a copy of the License at
#  *
#  *   http://www.apache.org/licenses/LICENSE-2.0
#  *
#  * Unless required by applicable law or agreed to in writing,
#  * software distributed under the License is distributed on an
#  * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
#  * KIND, either express or implied.  See the License for the
#  * specific language governing permissions and limitations
#  * under the License.
#  */

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """Generate a LaunchDescription for the Turtlebot3 navigation system with optional bag recording."""
    
    # Declare launch argument for bag recording
    bag_record = LaunchConfiguration('bag_record')

    # Define the LaunchDescription
    return LaunchDescription([
        # Declare launch argument for bag recording
        DeclareLaunchArgument(
            'bag_record',
            default_value='False',
            description='Flag to start the bag recording.'
        ),

        # Include Gazebo world launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(
                    'turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
            ])
        ),

        # Launch the navigator node
        Node(
            package='navigate_turtlebot', 
            executable='navigator',  
        ),

        # Conditionally execute bag recording based on the launch argument
        ExecuteProcess(
            condition=IfCondition(bag_record),
            cmd=['ros2', 'bag', 'record', '-a', '-x /camera.+'],
            shell=True
        )
    ])
