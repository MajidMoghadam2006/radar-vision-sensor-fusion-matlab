# radar-vision-sensor-fusion-matlab
Object-level sensor fusion using radar and vision synthetic data in MATLAB

This project is a simple implementation of the Aeberhard's PhD thesis [Object-Level Fusion for Surround Environment Perception in Automated Driving Applications](https://d-nb.info/113647157X/34). We use the MATLAB's Scenario Generator Toolbox to create a simple highway driving scenario with synthetic radar and vision observations. The Extended Kalman Filter has been implemented to propagate the vehicles' states to future. The projected state values are compared with the current measurements to perform the tracking.
