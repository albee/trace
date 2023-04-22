This is the nodelet that controls the target Astrobee for following a desired tumbling motion.

Given an desired initial attitude and angular velocity, this nodelet integrates the rotational dynamics (using ENVISAT's inertia tensor) and passes setpoints to Astrobee's default controller. From simulator tests, it looks like Astrobee's default controller is good enough to mimic the ENVISAT tumbling trajectories.

The nodelet is automatically launched in `sim_td.launch`. For unit testing, one can do `roslaunch target_coordinator target_coordinator.launch`. Then, in another console, run the python script `target_unit_test.py` in the `scripts` folder. This script automatically runs a simulator test case that compares the resulting Astrobee tumble to Roberto's DLR sample ENVISAT tumble (`Motion_Target.dat`). Bagged data is saved in the `data` folder, and the trajectory can be plotted via the MATLAB script.
