# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import subprocess
import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

BASE_NAMESPACE = 'experiment/luci_1'
CAMERA_NAME = 'd435i'

def detect_accel_fps():
    """ dynamically queries the RealSense hardware to find the supported accelerometer FPS """
    try:
        res = subprocess.run(
            ['rs-enumerate-devices'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=3.0
        )
        if res.returncode == 0:
            # Split the entire console output into separate lines
            for line in res.stdout.splitlines():
                # Focus strictly on lines detailing the Accelerometer stream
                if "Accel" in line:
                    if "250" in line:
                        return 250
                    elif "200" in line:
                        return 200
    except Exception as e:
        print(f"[CUSTOM] Could not auto-detect RealSense IMU via rs-enumerate-devices: {e}")
    # Return default if auto-detect fails
    return 250

def generate_launch_description():
    """Launch file which brings up visual slam node configured for RealSense."""
    # Detect the correct hardware rate
    accel_fps = detect_accel_fps() # [63/250] or [100/200] depending on IMU chip
    gyro_fps = 400 # [200/400] for any IMU chip

    print(f"[CUSTOM] Automatically selected 'accel_fps': {accel_fps}")

    realsense_camera_node = Node(
        name=CAMERA_NAME,  # 'camera',
        namespace=BASE_NAMESPACE,  # '',  # 'camera'
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_color': True,
            'enable_depth': False,
            'pointcloud.enable': False, 
            'depth_module.emitter_enabled': 0,
            'depth_module.profile': '640x480x30',
            # 'depth_module.infra_profile': '480x270x60',
            'rgb_camera.profile': '640x480x15',
            'enable_gyro': True,
            'enable_accel': True,
            'gyro_fps': gyro_fps,
            'accel_fps': accel_fps,
            'unite_imu_method': 2,
            'camera_namespace': f'{BASE_NAMESPACE}/{CAMERA_NAME}',
            # 'camera_name': CAMERA_NAME,
        }],
    )

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        namespace=BASE_NAMESPACE,
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'enable_image_denoising': False,
            'rectified_images': True,
            # 'enable_ground_constraint_in_odometry': True, 
            'enable_imu_fusion': True,
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': float(max(accel_fps, gyro_fps)), # match the faster sensor
            'image_jitter_threshold_ms': 40., # NVIDIA has 22 ms for 90 FPS (~11 ms)
            'base_frame': 'camera_link',
            'imu_frame': 'camera_gyro_optical_frame',
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'camera_optical_frames': [
                'camera_infra1_optical_frame',
                'camera_infra2_optical_frame',
            ],
        }],
        remappings=[
            ('visual_slam/image_0', f'{CAMERA_NAME}/infra1/image_rect_raw'),
            ('visual_slam/camera_info_0', f'{CAMERA_NAME}/infra1/camera_info'),
            ('visual_slam/image_1', f'{CAMERA_NAME}/infra2/image_rect_raw'),
            ('visual_slam/camera_info_1', f'{CAMERA_NAME}/infra2/camera_info'),
            ('visual_slam/imu', f'{CAMERA_NAME}/imu'),
        ],
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace=BASE_NAMESPACE,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    return launch.LaunchDescription([
        visual_slam_launch_container, 
        realsense_camera_node
        ])
