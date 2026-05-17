# AGV Bot ROS 2 Workspace
 
 This workspace contains the software, robot description, LiDAR driver integration, and web dashboard assets for the AGV robot project.
 
 ## Workspace Layout
 
 ```text
 ros_ws/
 ├── src/
 │   ├── agv_description/          # URDF, Gazebo, RViz, Nav2 and bridge configuration
 │   ├── agv_test_pkg/             # Python nodes, ODrive dashboard, teleop and navigation helpers
 │   ├── ydlidar_ros2_driver/      # YDLIDAR ROS 2 driver package
 │   └── YDLidar-SDK/              # Vendor SDK source
 ├── docs/
 │   ├── calibration/             # ODrive and wheel calibration documentation
 │   └── plans/                   # Design and implementation planning documents
 ├── prompts/                     # Reusable operator/agent prompts
 ├── artifacts/
 │   └── graphs/                  # Generated graphviz and exported graph artifacts
 ├── build/                       # colcon build output
 ├── install/                     # colcon install output
 └── log/                         # colcon/runtime logs
 ```
 
 ## Package Responsibilities
 
 - **`agv_description`**
   Robot model, simulation world, RViz config, Gazebo bridge config, and Nav2 parameterization.
 
 - **`agv_test_pkg`**
   Runtime Python nodes for AGV experiments, ODrive control, navigation bridge logic, and the web dashboard.
 
 - **`ydlidar_ros2_driver`**
   ROS 2 LiDAR driver package and model-specific parameter files.
 
 ## Key Runtime Entry Points
 
 - **Robot dashboard**
   `src/agv_test_pkg/agv_test_pkg/odrive_dashboard.py`
 
 - **Simulation launch**
   `src/agv_description/launch/sim.launch.py`
 
 - **Robot description display**
   `src/agv_description/launch/display.launch.py`
 
 - **LiDAR driver launch**
   `src/ydlidar_ros2_driver/launch/ydlidar_launch.py`
 
 ## Notes
 
 - Keep ROS 2 packages under `src/` unchanged unless package names, imports, launch references, and install rules are updated together.
 - Keep generated folders such as `build/`, `install/`, and `log/` out of manual source reorganization.
 - Store operator notes, prompts, and planning documents outside package roots unless they are required by package installation rules.
