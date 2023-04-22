# chaser_coordinator

A set of tools that handle feeding an input trajectory to the controller. Input can be in the form of a CSV, or direct call to the nodelet.

chaser_coordinator nodelet : reads in a pre-defined trajectory and sends off commands to the controller

# Usage:

See `execute_ASAP/` for usage instructions.

--

# Details

The nodelet is started from MLP.launch, located in astrobee/launch/robot/MLP.launch.
The nodelet_plugins.xml file had to be defined for this nodelet.
The chaser_coordinator::ChaserCoordinatorNodelet class had to be created, extending FreeFlyerNodelet.

`csv_read.cpp` has functions for reading in a (1) a normal CSV file and (2) DLR's Chaser and Target output directories. The
generated trajectory is an eigen matrix with the following information:

### Formatting for Read-In Information (Chaser)
[t x y z xd yd zd qx qy qz qw wx wy wz xdd ydd zdd wxd wyd wzd
...
]

### Formatting for Read-In Information (Target)
(TO DO)
