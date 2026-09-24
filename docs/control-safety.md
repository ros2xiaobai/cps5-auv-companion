# Control execution safety

`cps5-monitor` is read-only. It is the default entry point for checking MAVLink and DVL data.

`cps5-control` defaults to a dry-run that only validates YAML files and constructs the hardware-independent mission controller. Hardware execution is blocked unless all of the following are true:

1. `--execute` is supplied.
2. The three operator confirmation flags are supplied.
3. `allow_actuation`, `rc_mapping_validated`, `depth_mapping_validated`, and `dvl_axis_validated` are all `true` in a local, Git-ignored configuration.
4. The process has an interactive terminal and the operator types the exact final confirmation phrase.
5. Fresh heading, altitude and DVL position data are received before the arm command is sent.

The example configuration keeps every actuation permission false. Do not change those values merely to make the software start. They record completed physical checks.

The controller waits for a heartbeat reporting the configured flight mode before it requests arming. On exit it requests neutral outputs, disarming and release of all RC override channels. These software requests do not replace a physical emergency stop or an operator ready at QGroundControl. QGroundControl/joystick and the Python controller must not send competing pilot inputs during an automatic run; MAVLink Router forwards messages but does not arbitrate control ownership.

The refactored mission controller does not send the DVL reset command. It captures the current local x position as the origin of each segment, avoiding the prototype's competing reads from one TCP socket. This still depends on the installed DVL x axis being aligned with the commanded forward direction.

Automatic `roll`, `turn`, `photo`, `mod`, `a`, `b`, and `c` task actions from the prototype are deliberately rejected. They have not yet been migrated into independently gated and testable actions.
