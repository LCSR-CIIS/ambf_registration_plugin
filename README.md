# AMBF Registration Plugin
This document provides instructions and details for using the AMBF Registration Plugin, developed for integration with the Asynchronous Multibody Framework ([AMBF](https://github.com/WPI-AIM/ambf)) developed by Munawar et al.
this plugin facilitates various registration and calibration methods, including Pointer Base Registration, Point Cloud Registration, Hand-Eye calibration and Pivot Calibration.

https://github.com/user-attachments/assets/98d52432-d8ca-4ddc-8234-8a789d029b29

## 1. Installation Instructions:
Let's call the absolute location of this package as **<plugin_path>**. E.g. if you cloned this repo in your home folder, **<plugin_path>** = `~/ambf_registration_plugin/` OR `/home/<username>/ambf_registration_plugin`.

### 1.1 clone and build the repository
```bash
git clone git@github.com:LCSR-CIIS/ambf_registration_plugin.git
cd ambf_registration_plugin
mkdir build && cd
cmake ..
make
```
## 2 Preparation
### 2.0 Drill fiducial screws on the anatomy and segment scans
For Pin Base Registration, it's essential to rigidly attach CT-opaque fiducials that are clearly visible and reachable by the tool/drill. After obtaining the CT scan of the anatomy, segment it using  [3D Slicer](https://www.slicer.org/) and employ its [markups functionality](https://slicer.readthedocs.io/en/latest/user_guide/modules/markups.html) to save the fiducial points.

### 2.1 Generating ADF for AMBF
To generate ADF, please refer to [AMBF Slicer Plugin](https://github.com/LCSR-CIIS/ambf_util_slicer_plugin). You can also save the markups location and generate csv file.
Once you generated the anatomy, attach multiple points in the ADF file by editing the ADF by following lines:

''' ADF file
bodies:
- BODY Registering_anatomy
- BODY Registering_anatomy_origin
- BODY F1  #<- Add Points for the fiducials
- BODY F2  #<- Add Points for the fiducials
- BODY F3  #<- Add Points for the fiducials
- BODY F4  #<- Add Points for the fiducials
- BODY F5  #<- Add Points for the fiducials

'''
For each of the fiducial define the locations by adding following lines in the ADF file:
``` ADF file
BODY F01:
  name: F01
  mass: 0.001
  location:
    position: {x: 0.0, y: 0.0, z: 0.0} # CHANGE this value by markups.csv file.
    orientation: {r: 0.0, p: 0.0, y: 0.0 }
```

## 2. Registration/Calibration methods
### 2.1 Pin-Base method
The implementation details for the Pin Base (point-set) method are available in "src/point_cloud_registration.cpp". It is important to note that the order of input points and registering points is crucial (ORDER MATTERS).

This method assumes the use of a calibrated drill, which touches the fiducials and is tracked by an optical tracker in real-time.

In order to implement the plugin for moving the rigidBody in AMBF according to tracker reading, please refer to [ambf_tf_plugin](https://github.com/LCSR-CIIS/ambf_tf_plugin). 

### 2.2 Point Cloud Registration (Not currently used)
[Point Cloud Library(PCL)](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html) was used to implement Point Cloud Registration. We are currently not using this method, but code can be found in "src/point_cloud_registration.cpp". [Caution] Validation is recommended before use!!


### 2.3 Pivot Calibration
While pivoting the tool around the fiduical, we record the locations of the marker on the tool and the marker on the base with fiducial. 
You can also use the python code developed for [Twin-S](git@github.com:Rxliang/Twin-S.git) to try alternative way.

```bash
git clone git@github.com:Rxliang/Twin-S.git
cd Twin-S/optical_tracking   
python3 sksurgery_pivot_calibration.py -i ~/ambf_registration_plugin/data/Pivot_trackerTomarker.csv -c ../config/ransac_config.json

```

<!-- ![Pivot_calibration](/figs/) Add figure here to describe the hardware setup used for pivot calibration -->

### 2.4 Hand-Eye Calibration
To complete the whole kinematic chain up to the tip of the tool. Hand-Eye calibration is needed to calculate the frame transformation for the End-Effector(EE) frame to the marker on the tool.
We decided to use the pacakage developed by ETH team ([repo](https://github.com/ethz-asl/hand_eye_calibration)).
Please clone the repo using the following command:
```bash
git clone git@github.com:ethz-asl/hand_eye_calibration.git
```

<!-- ![Hand-eye_calibration](/figs/) Add figure here to describe the hardware setup used for pivot calibration -->



## 3. How to run the plugin
To add a volume to the scene, you can either include an ADF file in the launch file or specify it using the "-a <path_to_Anatomy_ADF_file>" option. Adjust the number after "-l" based on the desired ADF import for your AMBF setup.

```bash
ambf_simulator --launch_file launch.yaml -l 0,1,2,3 --registration_config example/registration_config.yaml
```

Command line options:
- `-l 0,1,2,3`: will add object defined in `launch.yaml`. The number corresponds with the index in `multibody configs:`. For examle, `1` is `registration_cube.yaml` and `2` is volume `marker.yaml`. 
- `--registration_config`: path to the registration configuration file.


### 3.0 Registration configuration file
Begin by creating a registration configuration file, using the example provided in "example/template_config.yaml".

``` registration_config.yaml
## Hand-Eye Regisration
hand eye:
  # Both the robot and the marker status should be published using CRTK
  robot namespace: /REMS/Research/ # namespace for the robot 
  marker namespace: /atracsys/Anspoch_drill # namespace for the marker
  referece namespace: /atracsys/Anatomy # [Optional] namespace for the reference
  EE name: dovetail_male
  marker name: marker_body # [Optional] Name of the rigid body for the marker body
  optical tracker name: Tracker # [Optional] Name of the rigid body for the tracker
  resolution: 0.001
  number of points: 1500
  registered HE result: # <- uncomment the following part if you finished Hand-Eye calibration
    q_rot: {x: -0.21229946, y: 0.8618833, z: 0.41732556, w: 0.19474468}
    q_dual: {x: -0.03906011, y: -0.02503096, z: 0.0335063, w: -0.00360324}

## Pivot calibration
pivot:
  marker namespace: /atracsys/Anspoch_drill # namespace for the marker
  referece namespace: /atracsys/Anatomy # [Optional] namespace for the reference
  marker name: marker_body # [Optional] Name of the rigid body in AMBF
  type: AUTO # AUTO: Reads the value all the time, MANUAL: Sample points
  resolution: 0.01
  number of points: 150
  registered pivot result: # <- uncomment the following part if you finished pivot calibration
    t_tip: {x: -0.077053, y: 0.150933, z: 0.043711}

## Pointer based registration
pointer:
  tooltip name: drill_tip # Rigid body that is attached to the tip of the tool
  name of points:
    - F1
    - F2
    - F3
    - F4
    - F5
  object name: TB04_anatomical_origin

```

You can modify the location/direction of the model camera by adding the following section in the registration configuration file.
```
## Model camera related region
  camera:
    location_offset: {x: 0.5, y: 0.0, z: -0.25} # Offset from the registering object
    look at: {x: 0.0, y: 0.0, z: -0.5} # Camera look at vector
    up: {x: 0.0, y: 0.0, z: 1.0} # Camera up vector
```


### 3.1 Perform Hand-Eye calibration
Press `[Ctrl + 1]` to activate Hand-Eye calibration mode.
Once there are enough new readings published via rostopic, it will automatically create a set of csv files under '/data'.
Use these files to execute Hand-Eye Calibration, then copy the results (q_rot, q_dual) into the registration configuration file.
```bash
cd hand_eye_calibration/hand_eye_calibration/bin
python3 compute_hand_eye_calibration.py --aligned_poses_B_H_csv_file ~/ambf_registration_plugin/data/HE_worldToEE.csv --aligned_poses_W_E_csv_file ~/ambf_registration_plugin/data/HE_trackerTomarker.csv --visualize VIZULALIZE
```
After completing the calibration, copy the result (t_tip) and incorporate it into the registration configuration file.


### 3.2 Perform Pivot calibration
Press `[Ctrl + 2]` to activate Hand-Eye calibration mode.
Once you are done with the pivot calibration. You can copy the result (t_tip) and paste in the registration_configuration file.

### 3.1 Perform Pin-base registration
Press `[Ctrl + 3]` to activate pin-base registration mode. Press `[Ctrl + 9]` to store the points.
[Caution] Sampling order matters!! Make sure to sample the points in the same order as the points in `launch.yaml`.
The calibration results will be printed in the terminal. Copy and paste the results in the ADF: 
```bash
position: {x: 0.0, y: 0.0, z: 0.0}
orientation: {r: 0.0, p:0.0, y: 0.0}
```
In the simulation, green points are the points defined in the registering model (ex. CT scan), red points are the points sampled by `[Ctrl + 9]`, and blue points are registered points.
https://github.com/user-attachments/assets/d14efbcf-4a87-4084-be2e-f6f2472448cc

## Trouble shooting
Please refer to [AMBF helper](https://github.com/LCSR-CIIS/AMBF_helper) for installation procedure and how to debug the plugins.
