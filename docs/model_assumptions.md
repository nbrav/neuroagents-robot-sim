# Robot source and assumptions

Source build: https://rebrickable.com/mocs/MOC-76666/Mr_Jos/6-degrees-of-freedom-6dof-fully-automatic-robotic-arm-with-inverse-kinematics-programmed/

Designer discussion: https://www.eurobricks.com/forum/forums/topic/182061-6-dof-robotic-arm-by-jos/

Published geometry/forward kinematics: https://github.com/Mr-Jos-Technic/Lego-EV3-6DoF/blob/main/6dof_left_accurate.py

The current model uses a shoulder height of 165 mm, upper-arm spacing of 168 mm, an elbow offset of 47 mm, 152 mm toward the wrist and a 115 mm wrist-to-tool reference. These are dimensions from one published revision, not measurements of your assembled arm. The tool distance includes the chosen reference point and should be changed for a pen or other attachment.

At zero joint angles, the upper arm is vertical, the forearm points forward and the tool reference is `(0.267, 0, 0.380)` metres. The geometry check compares XYZ against the published equations for 100 configurations. Full tool orientation, roll zero, encoder offsets and motor directions still require hardware calibration.

The robot uses simplified boxes, capsules and a fixed fork. Masses, inertias, damping, armature and torque bounds are estimates. Only the shoulder range approximately follows the cited code; the other bounded joint ranges are provisional. Some axes are modeled as continuous, though physical cabling may limit them. Collision detection excludes mounting parts that overlap by construction and is not a detailed model of every LEGO collision.

The hard case adds independently driven motor/gear effects. It does not reproduce the designer's coupled wrist transmission, exact EV3 onboard control, homing sensors, communication protocols or full frame flex. Its speed, noise, backlash and compliance values are illustrative output-equivalent parameters. Initial transmission preload holds the starting configuration. Transmission torque clipping represents a simplified safety/slip limit, and the symmetric torque–speed envelope has no regenerative braking model.

Before predicting hardware performance, identify the exact revision, motor placement, gearing/coupling, masses, usable travel, tool offset, encoder behavior and measured timing. Physical calibration is future work; the present project is suitable for explaining the controller and demonstrating two contrasting simulation conditions.
