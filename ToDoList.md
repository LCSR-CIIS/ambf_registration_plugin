# ToDoList


## registration_plugin.cpp/h
[] Create functionalities to add/edit the collected pooints
[x] Create a coloroverlay that shows the which point is being registered


## registration
- Add other methods for registration in the folder.
    - [x] PointCloud Registration -> Point set based registration
    - [x] Pivot Calibration
    - [x] Hand-Eye Calibration 

- Add improved version of registration method.
[] For pointer based registration, sample and pivot around the dimple. For each point, average during the sampling.

- Add Tracker and marker in the AMBF world
[x] Add moving functionality (-> ambf_tf_plugin??)


## general
[x] Add model view window.
[] Make small window within the main_camera rather than having seperate camera
[] Only render the CT model
[x] Move the model view camera to see the sampling points -> Mouse motion
[x] Change the color of the point you are sampling
[x] Attach the light on the camera
[] Use cmultipoints rather than cShere for the visualizing points.

Slicer
[] Add fiducial output functionality