# Calibration with a flat calibration object

## Important commands
- Start iiwa
    - Either with the standard tool

          roslaunch tum_iiwa_zimmer_r840_moveit moveit_planning_execution.launch sim:=false iiwa_sensor:=hokuyo_utm30lx

    - Or in case of initial calibration with the calibration pin model
        
          roslaunch tum_iiwa_calibration_pin_moveit moveit_planning_execution.launch sim:=false

- Start Hokuyo Sensor

        roslaunch tum_workcell_launch hokuyo_utm_30lx_ew.launch
        roslaunch tum_workcell_launch sick_tim571_2050101.launch

- User confirmation (confirmation during teaching of calibration object coordinate system)

        rostopic pub -1 /user_confirmation calibration_flat/UserConfirmation "userConfirmation: true"
        
- Start Calibration Node
        
         roslaunch calibration_flat arm_to_laser_scanner_calibration_hokuyo.launch
         roslaunch calibration_flat arm_to_laser_scanner_calibration_sick.launch
         
- Hand Guiding Client

        rosrun actionlib axclient.py activate_hand_guiding hand_guiding/HandGuidingAction

- Action Client Calibration

         rosrun actionlib axclient.py arm_to_laser_scanner_calibration calibration_flat/ArmToLaserScannerCalibrationAction


## Calibration

1. Initial Calibration
    * Robot moving to home position
    * Activate hand guiding mode
    * Move the robot arm to the calibration points p1, p2, p3 on the calibration object and confirm each point (Topic /user_confirmation)
    * Deactivate hand guiding mode
    * Calculate homogeneous transformation base frame to calibration object frame
    * Save transformation matrix in target_directory  
   
2. Collect Scan Data
    * Move to scan position 1
    * Collect laser scan data (~200 meas.)
    * Move to scan position 2
    * Collect laser scan data
    * Process laser scan data (averaging over collected scans)
    
3. Extracting the significant points
    * Limitation of observed angle area (scan)
    * Transformation of measured ranges into xy-coordinates of laser scanner frame
    * Depth limitation
    * Defining line model parameters
    * RANSAC and Least Square (LS) optimization
    * Collecting of edge points p1 to p6 used for homogeneous transform calibration object frame into laser scanner frame

4. Determine the Calibration Matrix
    * Determine homogeneous transformation matrix base frame to tool frame
    * Calculate homogeneous transformation matrix laser scanner frame to calibration object frame (formulas of paper)
    * Calculate homogeneous transformation matrix tool frame to laser scanner (final result)
    * Save transformation matrix tool frame to laser scanner frame in target_directory


## Parameters

- autostart
    - typ: boolean
    - description: If selected, the calibration process starts after 3 seconds after the calibration node is started.
- initial_calibration
    - typ: boolean 
    - description: If selected, the initial calibration is performed. If not selected, the saved transformation matrix base to calibration object is used.
- width_calib_obj
    - typ: float
    - description: Width (in meter) of the calibration object
- angle_calib_obj
    - typ: float
    - description: Angle (in degree) of the cutout on the calibration object.
- delta_a_calib_obj
    - typ: float
    - description: Distance (in meter) between the point p1 on the calibration object and the cutout corner.
- offset_alpha_z
    - typ: double
    - description: Rotation (in degree) of end effector to scan tilted around the z-axis onto the calibration object
- offset_pre_scan
    - typ: double
    - description: Position (in meter) of end effector over scan positions in the z-axis
- near_distance
    - typ: float
    - description: Distance (in meter) during the first laser scan between the calibration object and robot arm end effector in Y axis of the calibration object.
- far_distance
    - typ: float
    - description: Distance (in meter) during the second laser scan between the calibration object and robot arm end effector in Y axis of the calibration object.
- calib_obj_offset_z
    - typ: float
    - description: Distance (in meter) during the first and second laser scan between the laser scanner center and the calibration object frame (Estimation)
- trans_ee_l
    - typ: double
    - description: Distance (in meter) between laser scanner z-axis and laser scanner center (Estimation)
- Pin length
    - typ: double
    - description: Distance (in meter) from end effector plane to pin peak
- number_of_scans
    - typ: int
    - description: Number of laser scans evaluated per shot.
- number_of_iterations_ransac_ls
    - typ: int
    - description: Number of tries for Ranscac and least squares to find optimal model paramters
- ransac_threshold
    - typ: double
    - description: Distance (in meter) of a point to the line model estimation performed by ransac
- tolerance inlier
    - typ: double
    - description: Distance (in meter) of final edge points of calibration object towards fitted model by ransac/leas squares
- home_pos_x
    - typ: float
    - description: X coordinate of the robot home position.
- home_pos_y
    - typ: float
    - description: X coordinate of the robot home position.
- home_pos_z
    - typ: float
    - description: Z coordinate of the robot home position.
- target_directory
    - typ: string
    - description: Destination folder in which the homogeneous transformation matrices are stored.