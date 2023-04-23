# data

You've found the right place to get and dump your data!

Inputs (like sample trajectories) are placed in `data/input/`.

Outputs (in simulation) should be placed in `data/output/`. Actual output on Astrobee hardware differs.

## Creating Reference Trajectories

You can create your own reference trajectories in Caroline's output format using

`scripts/create_des_traj`

`scripts/des_traj_to_caroline_format.m`

See `main_traj_demo_script.m` for a complete usage example.

## Visualizing Tumble Trajectories

You can viusalize TRACE trajectories using

`tumble_anim.m`

See `main_traj_demo_script.m` for a complete usage example.

## Rosbag Output
You can convert rosbag topics to .csv files by the following command:

`rostopic echo -b <name_of_bag_file> -p <topic> > <name_of_csv_file`

For instance, the following command will create a .csv file for the target EKF state:

`rostopic echo -b test2_awesome_rosbag.bag -p /gnc/ekf > JUN15_targ_ekf.csv`

There are some preliminary plotting tools in `bags/`.
