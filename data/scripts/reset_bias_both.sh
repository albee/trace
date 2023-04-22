#!/usr/bin/env bash
rosrun executive teleop_tool -reset_bias -ns="/honey/" &
rosrun executive teleop_tool -reset_bias -ns="/bumble/" &