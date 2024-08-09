# Velocity MUX

## Description
This package contains a node that acts as a multiplexer for command velocities
sent to the rover. It is used primarily for the Autonomous Navigation mission.
It subscribes to velocity commmands sent from both an attached joystick and the
rover's autonomy control stack, then forwards the command from the appropriate
source depending on the rover's current mode (autonomous or teleoperation).