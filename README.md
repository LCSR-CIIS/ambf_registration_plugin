# AMBF Registration Plugin
AMBF simulation plugin to perform registration related task.


## Point Cloud Registration
Point Cloud Library(PCL) was used to perform Point Cloud Registration.
/url ()


https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html


## How to run 
```bash
ambf_simulator --launch_file launch.yaml -l 0,1 -a ~/sdf_virtual_fixture/ADF/Galen/galen_stryker_tilted.yaml --registration_config example/registration_config.yaml
```

## Perform HandEye calibration
ETH HandEye calibration method
```bash
python3 compute_hand_eye_calibration.py --aligned_poses_B_H_csv_file ~/ambf_registration_plugin/data/HE_worldToEE.csv --aligned_poses_W_E_csv_file ~/ambf_registration_plugin/data/HE_trackerTomarker.csv --visualize VIZULALIZE
```