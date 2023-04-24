# Backup Controller

This nodelet is a backup PD controller for following a trajectory given to the Chaser.

# Running the Node
1. replace LLP.launch line 59 with:   `<include file="$(find trace_astrobee_interface)/launch/ff_nodelet.launch" if="$(eval arg('ns')=='')">`
2. rosrun backup_controller run_backup_node
3. rosrun backup_controller fake_setpoint
4. rosrun execute_ASAP execute_asap.py
5. Should move 0.4 in the x direction, ISS frame
