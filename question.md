Title:

Description:
We are trying to run a fake hardware test to verify that the drivers are running before we try interfacing with our UR3e. We are getting this error. It seems to say the action groups are not connected. 


```
root@7ec1b4bac5e7:/workspace# ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
[INFO] [launch]: All log files can be found below /root/.ros/log/2022-09-26-20-33-17-907107-7ec1b4bac5e7-63
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [move_group-1]: process started with pid [70]
[INFO] [rviz2-2]: process started with pid [72]
[INFO] [servo_node_main-3]: process started with pid [74]
[rviz2-2] QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root'
[servo_node_main-3] [WARN] [1664224398.778611607] [moveit_servo.servo_node]: Intra-process communication is disabled, consider enabling it by adding: 
[servo_node_main-3] extra_arguments=[{'use_intra_process_comms' : True}]
[servo_node_main-3] to the Servo composable node in the launch file
[move_group-1] [WARN] [1664224398.784076989] [move_group.move_group]: Falling back to using the the move_group node namespace (deprecated behavior).
[servo_node_main-3] [INFO] [1664224398.784146913] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.00306237 seconds
[servo_node_main-3] [INFO] [1664224398.784169576] [moveit_robot_model.robot_model]: Loading robot model 'ur'...
[servo_node_main-3] [INFO] [1664224398.784178353] [moveit_robot_model.robot_model]: No root/virtual joint specified in SRDF. Assuming fixed joint
[move_group-1] [INFO] [1664224398.786193715] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.00196971 seconds
[move_group-1] [INFO] [1664224398.786213373] [moveit_robot_model.robot_model]: Loading robot model 'ur'...
[move_group-1] [INFO] [1664224398.786219475] [moveit_robot_model.robot_model]: No root/virtual joint specified in SRDF. Assuming fixed joint
[servo_node_main-3] [WARN] [1664224398.794568411] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[move_group-1] Link base_link had 2 children
[move_group-1] Link base had 0 children
[move_group-1] Link base_link_inertia had 1 children
[move_group-1] Link shoulder_link had 1 children
[move_group-1] Link upper_arm_link had 1 children
[move_group-1] Link forearm_link had 1 children
[move_group-1] Link wrist_1_link had 1 children
[move_group-1] Link wrist_2_link had 1 children
[move_group-1] Link wrist_3_link had 2 children
[move_group-1] Link flange had 1 children
[move_group-1] Link tool0 had 0 children
[move_group-1] Link ft_frame had 0 children
[servo_node_main-3] [INFO] [1664224398.823172199] [moveit_ros.current_state_monitor]: Listening to joint states on topic '/joint_states'
[servo_node_main-3] [INFO] [1664224398.826699627] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/attached_collision_object' for attached collision objects
[servo_node_main-3] [INFO] [1664224398.826712772] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[servo_node_main-3] [INFO] [1664224398.827577319] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[move_group-1] [INFO] [1664224398.827798553] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Publishing maintained planning scene on 'monitored_planning_scene'
[move_group-1] [INFO] [1664224398.827905197] [moveit.ros_planning_interface.moveit_cpp]: Listening to 'joint_states' for joint states
[servo_node_main-3] [INFO] [1664224398.827925987] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Publishing maintained planning scene on '/servo_node/publish_planning_scene'
[move_group-1] [INFO] [1664224398.828467304] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[move_group-1] [INFO] [1664224398.828874063] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/attached_collision_object' for attached collision objects
[move_group-1] [INFO] [1664224398.828887599] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[move_group-1] [INFO] [1664224398.829120065] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[move_group-1] [INFO] [1664224398.829131607] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[move_group-1] [INFO] [1664224398.829391465] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[move_group-1] [INFO] [1664224398.829632276] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[move_group-1] [WARN] [1664224398.829951948] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[move_group-1] [ERROR] [1664224398.829961998] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[move_group-1] [INFO] [1664224398.832145893] [moveit.ros_planning_interface.moveit_cpp]: Loading planning pipeline 'move_group'
[servo_node_main-3] [WARN] [1664224398.833690563] [moveit_servo.servo_calcs]: No kinematics solver instantiated for group 'ur_manipulator'. Will use inverse Jacobian for servo calculations instead.
[servo_node_main-3] [WARN] [1664224398.833707856] [moveit_servo.collision_check]: Collision check rate is low, increase it in yaml file if CPU allows
[move_group-1] [INFO] [1664224398.840187982] [moveit.ros_planning.planning_pipeline]: Using planning interface 'OMPL'
[move_group-1] [INFO] [1664224398.841788939] [moveit_ros.add_time_optimal_parameterization]: Param 'move_group.path_tolerance' was not set. Using default value: 0.100000
[move_group-1] [INFO] [1664224398.841797886] [moveit_ros.add_time_optimal_parameterization]: Param 'move_group.resample_dt' was not set. Using default value: 0.100000
[move_group-1] [INFO] [1664224398.841801263] [moveit_ros.add_time_optimal_parameterization]: Param 'move_group.min_angle_change' was not set. Using default value: 0.001000
[move_group-1] [INFO] [1664224398.841810871] [moveit_ros.fix_workspace_bounds]: Param 'move_group.default_workspace_bounds' was not set. Using default value: 10.000000
[move_group-1] [INFO] [1664224398.841821341] [moveit_ros.fix_start_state_bounds]: Param 'move_group.start_state_max_bounds_error' was set to 0.100000
[move_group-1] [INFO] [1664224398.841825369] [moveit_ros.fix_start_state_bounds]: Param 'move_group.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-1] [INFO] [1664224398.841834065] [moveit_ros.fix_start_state_collision]: Param 'move_group.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-1] [INFO] [1664224398.841838013] [moveit_ros.fix_start_state_collision]: Param 'move_group.jiggle_fraction' was not set. Using default value: 0.020000
[move_group-1] [INFO] [1664224398.841841690] [moveit_ros.fix_start_state_collision]: Param 'move_group.max_sampling_attempts' was not set. Using default value: 100
[move_group-1] [INFO] [1664224398.841850016] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Add Time Optimal Parameterization'
[move_group-1] [INFO] [1664224398.841853192] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Workspace Bounds'
[move_group-1] [INFO] [1664224398.841855677] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Bounds'
[move_group-1] [INFO] [1664224398.841858813] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State In Collision'
[move_group-1] [INFO] [1664224398.841884562] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Path Constraints'
[move_group-1] [INFO] [1664224398.861999595] [moveit.plugins.moveit_simple_controller_manager]: Added FollowJointTrajectory controller for scaled_joint_trajectory_controller
[move_group-1] [INFO] [1664224398.863328742] [moveit.plugins.moveit_simple_controller_manager]: Added FollowJointTrajectory controller for joint_trajectory_controller
[move_group-1] [INFO] [1664224398.863419155] [moveit.plugins.moveit_simple_controller_manager]: Returned 2 controllers in list
[move_group-1] [INFO] [1664224398.863432841] [moveit.plugins.moveit_simple_controller_manager]: Returned 2 controllers in list
[move_group-1] [INFO] [1664224398.863812539] [moveit_ros.trajectory_execution_manager]: Trajectory execution is not managing controllers
[move_group-1] [INFO] [1664224398.863825283] [move_group.move_group]: MoveGroup debug mode is ON
[move_group-1] [INFO] [1664224398.874847342] [move_group.move_group]: 
[move_group-1] 
[move_group-1] ********************************************************
[move_group-1] * MoveGroup using: 
[move_group-1] *     - ApplyPlanningSceneService
[move_group-1] *     - ClearOctomapService
[move_group-1] *     - CartesianPathService
[move_group-1] *     - ExecuteTrajectoryAction
[move_group-1] *     - GetPlanningSceneService
[move_group-1] *     - KinematicsService
[move_group-1] *     - MoveAction
[move_group-1] *     - MotionPlanService
[move_group-1] *     - QueryPlannersService
[move_group-1] *     - StateValidationService
[move_group-1] ********************************************************
[move_group-1] 
[move_group-1] [INFO] [1664224398.874866799] [moveit_move_group_capabilities_base.move_group_context]: MoveGroup context using planning plugin ompl_interface/OMPLPlanner
[move_group-1] [INFO] [1664224398.874872931] [moveit_move_group_capabilities_base.move_group_context]: MoveGroup context initialization complete
[move_group-1] Loading 'move_group/ApplyPlanningSceneService'...
[move_group-1] Loading 'move_group/ClearOctomapService'...
[move_group-1] Loading 'move_group/MoveGroupCartesianPathService'...
[move_group-1] Loading 'move_group/MoveGroupExecuteTrajectoryAction'...
[move_group-1] Loading 'move_group/MoveGroupGetPlanningSceneService'...
[move_group-1] Loading 'move_group/MoveGroupKinematicsService'...
[move_group-1] Loading 'move_group/MoveGroupMoveAction'...
[move_group-1] Loading 'move_group/MoveGroupPlanService'...
[move_group-1] Loading 'move_group/MoveGroupQueryPlannersService'...
[move_group-1] Loading 'move_group/MoveGroupStateValidationService'...
[move_group-1] 
[move_group-1] You can start planning now!
[move_group-1] 
[rviz2-2] [INFO] [1664224399.685820108] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-2] [INFO] [1664224399.685896274] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[rviz2-2] [INFO] [1664224399.908795319] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-2] Warning: class_loader.impl: SEVERE WARNING!!! A namespace collision has occurred with plugin factory for class rviz_default_plugins::displays::InteractiveMarkerDisplay. New factory will OVERWRITE existing one. This situation occurs when libraries containing plugins are directly linked against an executable (the one running right now generating this message). Please separate plugins out into their own library or just don't link against the library and use either class_loader::ClassLoader/MultiLibraryClassLoader to open.
[rviz2-2]          at line 253 in /opt/ros/humble/include/class_loader/class_loader/class_loader_core.hpp
[rviz2-2] [ERROR] [1664224402.981357641] [moveit_ros_visualization.motion_planning_frame]: Action server: /recognize_objects not available
[rviz2-2] [INFO] [1664224402.992299955] [moveit_ros_visualization.motion_planning_frame]: MoveGroup namespace changed: / -> . Reloading params.
[rviz2-2] [INFO] [1664224403.058050126] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.00432583 seconds
[rviz2-2] [INFO] [1664224403.058093870] [moveit_robot_model.robot_model]: Loading robot model 'ur'...
[rviz2-2] [INFO] [1664224403.058109119] [moveit_robot_model.robot_model]: No root/virtual joint specified in SRDF. Assuming fixed joint
[rviz2-2] Link base_link had 2 children
[rviz2-2] Link base had 0 children
[rviz2-2] Link base_link_inertia had 1 children
[rviz2-2] Link shoulder_link had 1 children
[rviz2-2] Link upper_arm_link had 1 children
[rviz2-2] Link forearm_link had 1 children
[rviz2-2] Link wrist_1_link had 1 children
[rviz2-2] Link wrist_2_link had 1 children
[rviz2-2] Link wrist_3_link had 2 children
[rviz2-2] Link flange had 1 children
[rviz2-2] Link tool0 had 0 children
[rviz2-2] Link ft_frame had 0 children
[rviz2-2] [INFO] [1664224403.107834778] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[rviz2-2] [INFO] [1664224403.108357299] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/monitored_planning_scene'
[rviz2-2] [INFO] [1664224403.270823856] [interactive_marker_display_94801463168560]: Connected on namespace: /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic
[rviz2-2] Link base_link had 2 children
[rviz2-2] Link base had 0 children
[rviz2-2] Link base_link_inertia had 1 children
[rviz2-2] Link shoulder_link had 1 children
[rviz2-2] Link upper_arm_link had 1 children
[rviz2-2] Link forearm_link had 1 children
[rviz2-2] Link wrist_1_link had 1 children
[rviz2-2] Link wrist_2_link had 1 children
[rviz2-2] Link wrist_3_link had 2 children
[rviz2-2] Link flange had 1 children
[rviz2-2] Link tool0 had 0 children
[rviz2-2] Link ft_frame had 0 children
[rviz2-2] [INFO] [1664224403.275805340] [moveit_ros_visualization.motion_planning_frame]: group ur_manipulator
[rviz2-2] [INFO] [1664224403.275828965] [moveit_ros_visualization.motion_planning_frame]: Constructing new MoveGroup connection for group 'ur_manipulator' in namespace ''
[rviz2-2] [WARN] [1664224403.276076499] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[rviz2-2] [INFO] [1664224403.289162359] [move_group_interface]: Ready to take commands for planning group ur_manipulator.
[rviz2-2] [INFO] [1664224403.299046787] [interactive_marker_display_94801463168560]: Sending request for interactive markers
[rviz2-2] [INFO] [1664224403.333194890] [interactive_marker_display_94801463168560]: Service response received for initialization
[rviz2-2] [INFO] [1664224410.642978193] [move_group_interface]: MoveGroup action client/server ready
[move_group-1] [INFO] [1664224410.643492238] [moveit_move_group_default_capabilities.move_action_capability]: Received request
[move_group-1] [INFO] [1664224410.643685348] [moveit_move_group_default_capabilities.move_action_capability]: executing..
[move_group-1] [INFO] [1664224410.645987638] [moveit_move_group_default_capabilities.move_action_capability]: Planning request received for MoveGroup action. Forwarding to planning pipeline.
[move_group-1] [INFO] [1664224410.646058964] [moveit_move_group_capabilities_base.move_group_capability]: Using planning pipeline 'move_group'
[move_group-1] [INFO] [1664224410.646995508] [moveit.ompl_planning.model_based_planning_context]: Planner configuration 'ur_manipulator' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[move_group-1] [WARN] [1664224410.723960542] [moveit_trajectory_processing.time_optimal_trajectory_generation]: Joint acceleration limits are not defined. Using the default 1 rad/s^2. You can define acceleration limits in the URDF or joint_limits.yaml.
[move_group-1] [INFO] [1664224410.727644168] [moveit_move_group_default_capabilities.move_action_capability]: Motion plan was computed successfully.
[rviz2-2] [INFO] [1664224411.043897661] [move_group_interface]: Planning request accepted
[rviz2-2] [INFO] [1664224411.144159880] [move_group_interface]: Planning request complete!
[rviz2-2] [INFO] [1664224411.244289563] [move_group_interface]: time taken to generate plan: 0.0239999 seconds
[move_group-1] [INFO] [1664224412.987339226] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Received goal request
[move_group-1] [INFO] [1664224412.987458304] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution request received
[move_group-1] [INFO] [1664224412.987499343] [moveit.plugins.moveit_simple_controller_manager]: Returned 2 controllers in list
[move_group-1] [INFO] [1664224412.987521495] [moveit.plugins.moveit_simple_controller_manager]: Returned 2 controllers in list
[move_group-1] [INFO] [1664224412.987641174] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.01
[move_group-1] [INFO] [1664224412.989836229] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-1] [INFO] [1664224412.989900252] [moveit.plugins.moveit_simple_controller_manager]: Returned 2 controllers in list
[move_group-1] [INFO] [1664224412.989923726] [moveit.plugins.moveit_simple_controller_manager]: Returned 2 controllers in list
[move_group-1] [ERROR] [1664224412.990025692] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Action client not connected to action server: joint_trajectory_controller/follow_joint_trajectory
[move_group-1] [ERROR] [1664224412.990032926] [moveit_ros.trajectory_execution_manager]: Failed to send trajectory part 1 of 1 to controller joint_trajectory_controller
[move_group-1] [INFO] [1664224412.990039588] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status ABORTED ...
[move_group-1] [INFO] [1664224412.990114772] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution completed: ABORTED
[rviz2-2] [INFO] [1664224413.487858580] [move_group_interface]: Execute request accepted
[rviz2-2] [INFO] [1664224413.588066876] [move_group_interface]: Execute request aborted
[rviz2-2] [ERROR] [1664224413.688185922] [move_group_interface]: MoveGroupInterface::execute() failed or timeout reached
```