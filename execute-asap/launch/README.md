# TumbleDock nodelet launch files

The `execute_asap` node launches nodelets specific to the TumbleDock experiment upon recieving a test number from GDS. 
Based upon `execute_asap`'s role, either `td_chaser.launch` or `td_target.launch` is called.

From here, specific launch files for the robot's MLP and LLP actually launch each nodelet. Each nodelet is launched as a standalone nodelet, 
which offers the ability to kill and re-launch them for each test.

On hardware, the `casadi_nmpc` nodelet is launched on the LLP. As such, the LLP IP address is passed as an argument to the launch command.

Upon a -1 test command, all TD nodelets are killed. They are restarted upon receiving a new test command that is not -1.
