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