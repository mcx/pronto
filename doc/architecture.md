# Pronto Architecture

The system offers a set of **modules** that manage different sensors subscribing to specific topics.
Available modules:

1. `ins`: inertial navigation system (IMU)
2. `legodo`: legged odometry
3. `bias_lock`: bias updater while the system is standing
4. `scan_matcher`: LiDAR scan matcher
5. `qualysis_mt`: Qualisys motion tracker

---

### ROS2 Code UML

![Prova](/doc/pronto_node_ros1.png)

---

### Pronto Estimator Parameters

#### Estimator parameters

- `pose_topic`: estimated pose topic name
- `pose_frame_id`: pose `tf` frame name
- `twist_topic`: estimated twist topic name
- `publish_pose`: if `true`, publish pose
- `publish_tf`: if `true`, publish `tf`
- `tf_child_frame_id`
- `republish_sensors`
- `init_sensors`: sensor list used to initialize the filter
- `active_sensors`: sensor list used to update the estimation
- `utime_history_span`: maximum update temporal span
- `sigma0`: initial covariance
- `x0`: initial state

#### Base-module parameters

- `topic`: module subscription topic name
- `roll_forward_on_receive`: if `true`, update the estimation when a message is received
- `publish_head_on_message`: if `true`, publish the estimation when a message is received

#### INS parameters

- `q_gyro`: gyro measurement covariance
- `q_accel`: accelerometer measurement covariance
- `q_gyro_bias`: gyro bias covariance
- `q_accel_bias`: accelerometer bias covariance
- `num_to_init`: number of messages needed to initialize the INS module
- `gyro/accel_bias_initial`: initial bias value
- `gyro/accel_bias_recalc_at_start`: if `true`, recalculate bias during initialization
- `gyro/accel_bias_update_online`: if `true`, update bias online
- `frame`: IMU `tf` frame name
- `base_frame`: base `tf` frame name

#### Legodo parameters

- `legodo_mode`: covariance mode (`STATIC_SIGMA`, `VAR_SIGMA`, `IMPACT_SIGMA`, `WEIGHTED_AVG`, `ALPHA_FILTER`)
- `stance_mode`: contact-detection mode (`THRESHOLD`, `HYSTERESIS`, `REGRESSION`)
- `stance_threshold`
- `stance_hysteresis_low`
- `stance_hysteresis_high`
- `stance_hysteresis_delay_low`
- `stance_hysteresis_delay_high`
- `stance_alpha`
- `stance_regression_beta_size`
- `stance_regression_beta`
- `r_vx`: initial covariance in **x** direction
- `r_vy`: initial covariance in **y** direction
- `r_vz`: initial covariance in **z** direction
- `sim`: if `true`, use `sensor_msgs`; otherwise use `pi3hat_msgs`

#### Bias-lock parameters

- `torque_threshold`: minimum knee torque to detect ground contact
- `velocity_threshold`: maximum velocity to consider the robot standing
- `secondary_topic`: joint-state topic name
- `sim`: if `true`, use `sensor_msgs`; otherwise use `pi3hat_msgs`

#### Scan-matcher parameters

- `mode`: correction mode (`position`, `yaw`, `position_yaw`)
- `r_yaw`: yaw covariance
- `r_pxy`: position covariance

#### Wheel-odometry parameters

- `mode`: correction mode (`linear_velocity`, `angular_velocity`, `both`)
- `r_linear`: linear-velocity covariance
- `r_chi`: angular covariance

#### Qualysis-MT parameters

- `robot_name`: Qualisys rigid-body name
- `r_xyz`: position covariance
- `r_chi`: orientation covariance
- `mode`: correction mode (`position`, `yaw`, `position_yaw`, `orientation`, `position_orientation`)

---

### Classes and Methods

1. **`ROS_FrontEnd`** (Pronto ROS)
   - **Constructor**: reads topic names for pose, velocity and `tf` from ROS parameters; initialises filter state and covariance.
   - **`initializeState`**: reads initial pose and velocity from parameters and sets the initial state.
   - **`initializeCovariance`**: reads the initial covariance matrix and sets it.
   - **`initializeFilter`**: initializes the filter once all required sensor modules are ready.
   - **`areModulesInitialized`**: checks whether all sensors that need initialisation are ready.
   - **`addSensingModule`**: creates data structures (maps and topics) for a sensor.
   - **`InitCallback`**: callback to initialize a sensing module; destroys the init topic when done.
   - **`callback`**: subscriber callback that updates the estimation from incoming data.

2. **`InsHandlerRos`** (Pronto ROS) — manages the `InsModule` for the frontend.
   - **Constructor**: builds the `InsModule` and creates its topic.
   - **`ProcessMessage`**: receives an IMU message and returns the state and covariance update.
   - **`processMessageInit`**: processes an IMU message and returns whether initialization succeeded.

3. **`SensingModule`** (Pronto_Core) — virtual base class for a generic sensor.
   - **`ProcessMessage`** *(virtual)*: processes sensor data and returns the update.
   - **`processMessageInit`** *(virtual)*: handles initialization messages.

4. **`DualSensingModule`** (Pronto_Core) — virtual class for a module with two sensor inputs.
   - **`ProcessMessage`** *(virtual)*
   - **`processMessageInit`** *(virtual)*

5. **`InsModule`** (Pronto_Core)
   - **Constructor**: builds the inertial sensing module from the IMU configuration and IMU-to-body transformation.
   - **`ProcessMessage`**
   - **`processMessageInit`**
