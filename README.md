# VIO
Visual inertial odometry using stereo cameras.
Good starting point for visual inertial odometry.  

# Dependencies
- OpenCV
- Eigen

# Accuracy improvement using machine learning
The objective is to improve accuracy while keeping the algorithm lightweight. Bundle adjustment is a classical approach to improve localization accuracy, but it's too heavy for embedded system and there's no guarentee of convergence. Observe that since drift is unavoidable (even with bundle adjustment), the best we can do is to remove every possible offsets or biases inhenrently built in every single step of the VIO (camera calibration, feature detection and tracking) and leave the algorithm with Gausian noise only. To do that, instead of analytically characterizing each step's biases, we learn all of them at once by perform a linear regression of errors against system's state variable.

In the figure below, blue line is the VIO's position estimation error, red line is the prediction of that error using a simple linear regression with 12 independent variables (corresponding to camera's egomotion and features' distribution). There's a strong implication that machine learning can significantly improve VIO's position estimation without adding any computational burden.     
![linear regression](linear_regression.png)
