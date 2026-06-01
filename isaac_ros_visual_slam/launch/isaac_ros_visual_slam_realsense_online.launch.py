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

import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

camera_node_name = 'luci_1_d435i'  # 'camera'
camera_node_namespace = 'experiment/luci_1/luci_1_d435i'  # ''

def detect_accel_fps():
    """ dynamically queries the RealSense hardware to find the supported accelerometer FPS """
    try:
        import pyrealsense2 as rs
        ctx = rs.context()
        devices = ctx.query_devices()
        if devices:
            # Query first-available device
            for sensor in devices[0].query_sensors():
                if sensor.is_motion_sensor():
                    for profile in sensor.get_stream_profiles():
                        if profile.stream_type() == rs.stream.accel:
                            fps = profile.fps()
                            if fps in [200, 250]:
                                return fps
    except Exception as e:
        print(f"[Launch Warning] Could not auto-detect RealSense IMU via pyrealsense2: {e}")

    # Return default if auto-detect fails
    return 250

def generate_launch_description():
    """Launch file which brings up visual slam node configured for RealSense."""
    # Detect the correct hardware rate
    accel_fps_val = detect_accel_fps()
    print(f"[Launch Info] Automatically selected 'accel_fps': {accel_fps_val}")

    realsense_camera_node = Node(
        name=camera_node_name,  # 'camera',
        namespace=camera_node_namespace,  # '',  # 'camera',
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
            'gyro_fps': 400, # 200,
            'accel_fps': accel_fps_val,  # [63/250] or [100/200] depending on IMU chip
            'unite_imu_method': 2
        }],
    )

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
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
            'calibration_frequency': 200.0,
            'image_jitter_threshold_ms': 22.00,
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
            ('visual_slam/image_0', f'{camera_node_namespace}/infra1/image_rect_raw'),
            ('visual_slam/camera_info_0', f'{camera_node_namespace}/infra1/camera_info'),
            ('visual_slam/image_1', f'{camera_node_namespace}/infra2/image_rect_raw'),
            ('visual_slam/camera_info_1', f'{camera_node_namespace}/infra2/camera_info'),
            ('visual_slam/imu', f'{camera_node_namespace}/imu'),
        ],
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    return launch.LaunchDescription([
        visual_slam_launch_container, 
        realsense_camera_node
        ])
