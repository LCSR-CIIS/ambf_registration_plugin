# AMBF Registration Plugin
This document provides instructions and details for using the AMBF Registration Plugin, developed for integration with the Asynchronous Multibody Framework ([AMBF](https://github.com/WPI-AIM/ambf)) developed by Munawar et al.
this plugin facilitates various registration and calibration methodologies, including Pin Base Registration, Point Cloud Registration, Hand-Eye calibration and Pivot Calibration.

## 1. Installation Instructions:
Let's call the absolute location of this package as **<plugin_path>**. E.g. if you cloned this repo in your home folder, **<plugin_path>** = `~/ambf_registration_plugin/` OR `/home/<username>/ambf_registration_plugin`.

### 1.1 clone and build the repository
```bash
git clone git@github.com:LCSR-CIIS/ambf_registration_plugin.git
cd ambf_registration_plugin
mkdir build && cd 
make
```
## 2 Preparation
### 2.0 Drill fiducial screws on the anatomy and segment scans
For Pin Base Registration, it's essential to rigidly attach CT-opaque fiducials that are clearly visible and reachable by the tool/drill. After obtaining the CT scan of the anatomy, segment it using  [3D Slicer](https://www.slicer.org/) and employ its [markups functionality](https://slicer.readthedocs.io/en/latest/user_guide/modules/markups.html) to save the fiducial points.

### 2.1 Generating ADF for AMBF
To generate ADF, please refer to [AMBF Slicer Plugin](https://github.com/LCSR-CIIS/ambf_util_slicer_plugin). You can also save the markups location and generate this plugin.
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
BODY F1:
  name: F1
  mesh: Sphere.OBJ # Generate shpere in Blender or use any mesh.
  collision mesh type: CONCAVE_MESH
  mass: 0.0
  collision margin:
  scale: 1.0
  location:
    position: {x: -0.02749, y: -0.00057, z: -0.89238} 
    orientation: {r: 0.0, p: -0.0, y: 0.0}
  passive: true
  visible: true
  collision groups: []
  color components: # You can change the color here
    ambient:
      level: 1.0
    diffuse:
      b: 0.0
      g: 0.0
      r: 0.8
    specular:
      b: 0.0
      g: 0.0
      r: 0.0
    transparency: 1.0

```

## 2. Registration/Calibration methods
### 2.1 Pin-Base method
The implementation details for the Pin Base (point-set) method are available in "src/point_cloud_registration.cpp". It is important to note that the order of input points and registering points is crucial (ORDER MATTERS).

This method assumes the use of a calibrated drill, which touches the fiducials and is tracked by an optical tracker in real-time.

In order to implement the plugin for moving the rigidBody in AMBF according to tracker reading, please refer to [robot_control_plugin](https://github.com/LCSR-CIIS/sdf_virtual_fixture/blob/main/plugins/control/robot_control.cpp). 

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
ambf_simulator --launch_file launch.yaml -l 0,1 -a ~/sdf_virtual_fixture/ADF/Galen/galen_stryker_tilted.yaml --registration_config example/registration_config.yaml
```

Command line options:
- `-l 5, 7`: will add object defined in `launch.yaml`. The number corresponds with the index in `multibody configs:`. For examle, `5` is optical tracked drill and `7` is volume `TB0904`. 
- `--registration_config`: path to the registration configuration file.


### 3.0 Registration configuration file
Begin by creating a registration configuration file, using the example provided in "example/template_config.yaml".

``` registration_config.yaml
# AMBF Registration configuration file template 
# Author: Hisashi Ishida
# Date: 01.10.2024

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

## Optical Tracker Registration
optical tracker:
  namespace: /atracsys #rostopic name for the optical marker
  name of points:
    - anatomy

## Hand-Eye Regisration
hand eye:
  # Both the robot and the marker status should be published using CRTK
  namespace: /REMS/Research/ #namespace for the robot 
  marker name: atracsys/Anspoch_drill # This name should used for both rostopic name and object name in AMBF
  joint name: dovetail_male
  optical tracker name: Tracker
  resolution: 0.001
  number of points: 1500
  #registered HE result:   # <- uncomment the following part if you finished Hand-Eye calibration
  #  q_rot:{x: 0.0, y: 0.0, z: 0.0, w: 0.0}
  #  q_dual:{x: 0.0, y: 0.0, z: 0.0, w: 0.0}

## Pivot calibration
pivot:
  tooltip name: drill_tip
  namespace: /REMS/Research/ #namespace for the robot 
  marker name: atracsys/Anspoch_drill
  resolution: 0.01
  number of points: 150
  #registered pivot result: # <- uncomment the following part if you finished pivot calibration
  #  t_tip: {x: 0.0, y: 0.0, z: 0.0}

```

### 3.1 Perform Pin-base registration
Press `[Ctrl + 1]` to activate pin-base registration mode. Press `[Ctrl + 9]` to store the points.
[Caution] Sampling order matters!! Make sure to sample the points in the same order as the points in `launch.yaml`.
The calibration results will be printed in the terminal. Copy and paste the results in the ADF: 
```bash
position: {x: 0.0, y:0.0, z:0.0}
orientation: {r: 0.0, p: 0.0, y:0.0}
```


### 3.2 Perform Hand-Eye calibration
Press `[Ctrl + 2]` to activate Hand-Eye calibration mode.
Once there are enough new readings published via rostopic, it will automatically create a set of csv files under '/data'.
Use these files to execute Hand-Eye Calibration, then copy the results (q_rot, q_dual) into the registration configuration file.
```bash
cd hand_eye_calibration/hand_eye_calibration/bin
python3 compute_hand_eye_calibration.py --aligned_poses_B_H_csv_file ~/ambf_registration_plugin/data/HE_worldToEE.csv --aligned_poses_W_E_csv_file ~/ambf_registration_plugin/data/HE_trackerTomarker.csv --visualize VIZULALIZE
```
After completing the calibration, copy the result (t_tip) and incorporate it into the registration configuration file.


### 3.3 Perform Pivot calibration
Press `[Ctrl + 2]` to activate Hand-Eye calibration mode.
Once you are done with the pivot calibration. You can copy the result (t_tip) and paste in the registration_configuration file.


## Experiment 

Change object name in "registration_config"
```bash
ambf_simulator --launch_file launch.yaml -l 4,7 --registration_config example/registration_config.yaml #TB0904
ambf_simulator --launch_file launch.yaml -l 4,8 --registration_config example/registration_config.yaml #TB0909

ambf_simulator --launch_file launch.yaml -l 4,10 --registration_config example/registration_config.yaml #DS0913_1
ambf_simulator --launch_file launch.yaml -l 4,11 --registration_config example/registration_config.yaml #DS0913_2

ambf_simulator --launch_file launch.yaml -l 4,14 --registration_config example/registration_config.yaml #DS0913_2

```

## Trouble shooting
Please refer to [AMBF helper](https://github.com/LCSR-CIIS/AMBF_helper) for installation procedure and how to debug the plugins.