Function to determine uncertainty bounds on the chaser trajectory in the inertial frame for designing the tube-MPC controller.

For unit testing, a trajectory from Roberto's look-up-table is used (parameter #4) along with a "dummy estimator" nodelet that publishes an example initial target state. The UC bound nodelet takes in the trajectory, target state estimate, and estimate covariance as its parameters. Monte Carlo trials are performed by randomly perturbing the target state estimate within a 3-sigma bound defined by the covariance. The chaser state uncertainty bound is taken as the max/min difference between the nominal and real chaser trajectories, specifically at the point of greatest error in the propagated tumble using the nominal/real initial state.

To perform the unit test: run `roslaunch uc_bound uc_bound.launch` and then in another console, run the script `uc_bound_unit_test.py` in the `scripts` folder.

The resulting bound seems a bit high (too conservative) for the unit test. But it's difficult to get a feel for its accuracy right now without real target state estimates/covariances.

Current settings:
1. 1000 Monte Carlo trials
2. 3-sigma bounds for random perturbation (based off of the estimate covariance)

TODO:
1. Fix uc_bound calculation for ISS.

## Usage

Run `uncertainty_bound.m`

Output is produced as the `uc_bound` workspace variable.
