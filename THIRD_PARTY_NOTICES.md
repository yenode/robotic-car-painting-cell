# Third-party assets

## Car

OpenRobotics, **Hatchback**, Gazebo Fuel version 3, CC0-1.0.

Source: https://fuel.gazebosim.org/1.0/OpenRobotics/models/Hatchback/3/Hatchback.zip

The original mesh is retained as `hatchback.obj`; `car_scaled.obj` is rotated, centered, grounded, and scaled for this assignment. Texture references were changed to relative local paths. The archive checksum is recorded in `src/painting_cell/models/hatchback/SOURCE.json`.

## Robot

Universal Robots, **Universal_Robots_ROS2_Description**, Jazzy branch, package version 3.5.1, downloaded 6 October 2026.

Source: https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/jazzy

The files required for UR20 are retained under `src/ur_description`; unrelated robot variants, documentation, and tests were removed to keep the submission small. The BSD-3-Clause license and separate mesh terms remain included. UR20 meshes are governed by `src/ur_description/meshes/ur20/LICENSE.txt`; do not relicense them under the project code license.

© 2023 Universal Robots A/S. Use hereof is subject to Universal Robots A/S’ Terms and Conditions for Use of Graphical Documentation.

The custom robot wrapper calls the upstream xacro macros; robot geometry has not been resized or modified.
