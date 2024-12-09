#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # package_dir = get_package_share_directory('px4_offboard')
    package_dir = os.path.join(os.getenv('HOME'), 'ws/src/px4_offboard')
    # self.get_logger().info('package_dir: {}'.format(package_dir))


    ld=LaunchDescription()


    
    visualizer=Node(
        package='px4_offboard',
        executable='visualizer',
        name='visualizer'
    )
    
    control=Node(
        package='px4_offboard',
        executable='control',
        name='control',
        prefix='gnome-terminal --'
    )

    joy=Node(
        package='joy',
        executable='joy_node'
    )

    rviz= Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
    )

    # gnome_term=ExecuteProcess(
    #     cmd=["gnome-terminal","--","/root/ws/src/px4_offboard/launch/gnome-tab"
    #     ]
    # )  
    
    gazebo=ExecuteProcess(
        cmd=["gnome-terminal","--","/root/ws/src/px4_offboard/launch/gnome-tab"
        ]
    )
    
    supervision=Node(
        package='px4_offboard',
        executable='supervision'
        # cmd=["python3","/root/ws/src/supervision.py"]
    )

    control_interface=Node(
        package='px4_offboard',
        executable='control_interface',
        # cmd = ["python3","/root/ws/src/px4_offboard/px4_offboard/control_interface.py"]
    )

    
    # microxrce=ExecuteProcess(
    #     # cmd=["MicroXRCEAgent","udp4","-p", "8888"]
    #     cmd = ["xterm", "-e", "bash -c 'cd ~ && MicroXRCEAgent udp4 -p 8888'"]
    # )
 
    ## px4 autopilot tab
    # px4_autopilot = ExecuteProcess(
    #     # cmd=["gnome-terminal", "--", "bash", "-c", "cd ~/PX4-Autopilot/ && make px4_sitl gazebo-classic_x500"]
    #     cmd=["xterm", "-e", "bash -c 'cd ~/PX4-Autopilot/ && make px4_sitl gazebo-classic_x500'"]

    # )
 


    
    px4_microxrce_offboard = ExecuteProcess(
        cmd=["xterm", "-e", "bash -c 'tmux new-session -d -s my_session \"cd ~ && MicroXRCEAgent udp4 -p 8888\"; tmux split-window -h \"cd ~/PX4-Autopilot/ && make px4_sitl gazebo-classic_x500\"; tmux select-pane -t 0; tmux split-window -v \"ros2 run px4_offboard control\"; tmux -2 attach-session -d'"]
    )


  
    ld.add_action(gazebo)    
    ld.add_action(joy)
    ld.add_action(supervision)
    ld.add_action(px4_microxrce_offboard)
    ld.add_action(control_interface)
    
    
    #pour visualiser pos/traj
    # ld.add_action(visualizer)
    # ld.add_action(rviz)
    
    

    return ld
