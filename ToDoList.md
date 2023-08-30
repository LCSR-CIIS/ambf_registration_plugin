# ToDoList

## Blender
- Check the Sphere_00 and Sphere_19 and their joints


## registration_plugin.cpp/h
- Create functionalities to add/edit the collected pooints
- Create a coloroverlay that shows the which point is being registered


## registration
- Add other methods for registration in the folder.
    - (Done) PointCloud Registration
    - (Done) Pivot Calibration
    - (In progress)Hand-Eye Calibration

- Add Tracker and marker in the AMBF world
These objects should be rigidBody. Need to be seperate ADF file. (Need to play with Blender)
-> Need to match the origin and the frame (May be for later???)
-> Change the geometry file can be easier??


## general
- camera_panel_manager.cpp/h and CRTKInterface.cpp/h should be in the ambf main branch to avoid local copies.

- CRTKInterface
 Check whether you are connected to the robot or not. (Status??)
 Galen does not publish meaning full topic now. Maybe we can use "REMS/Research/footpedal" value instead?


 ## ToDo
 - (Done) Add tracker for the anatomy
 - (Done) Check the ADF file for the robot
 - (Done) Verify the location of the tooltip
 - (Done) Redo the Hnadeye Calibration and pivot calibration
 - try Point Coud registration
 - Get/Print the calibratied transformation for the tooltip
 - Get the screws for the anatomy

 - Test Pivot registration
 - Test point-set registration

