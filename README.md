# MOOS-pNavEKF
An implementation of an Extended Kalman Filter for 2D navigation

This is based on Steven Levy's excellent Extended Kalman Filter tutorial (https://simondlevy.academic.wlu.edu/kalman-tutorial/) and implemented with 
librobotcontrol's EKF implementation.

It expects to receive current X (northing) and Y (easting) local coordinates from the GPS or a similar source along with true GPS heading in degrees and GPS computed velocity 
in meters per second. It expects to receive heading, yaw rate, and forward acceleration from an IMU. It then fuses these into a continuous position estimate. 

Future iterations will have the capacity to use local coordinates with a shifting origin point, but that has not yet been implemented. 
