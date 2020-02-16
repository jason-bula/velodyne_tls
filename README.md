# velodyne_tls

This  study  develops  a  method  to  acquire  densepoint  clouds  with  a  low-cost  Velodyne  VLP-16  lidar  system,without using expensive GNSS positioning or IMU. Our settingconsists  in  mounting  the  lidar  on  a  motor  to  continuouslychange the scan direction, which leads to a significant increasein  the  point  cloud  density.  A  post-treatment  reconstructs  theposition of each point accounting for the motor angle at the timeof acquisition, and a calibration step accounts for inaccuraciesin  the  hardware  assemblage.  The  system  is  tested  in  indoorssettings  such  as  buildings  and  abandoned  mines,  but  is  alsoexpected  to  give  good  results  outdoors. 



Instructions to run the codes

- First, download the example point cloud at this link: https://www109.zippyshare.com/v/EbxdH7My/file.html
- The .pcap file must be in the same folder as main_demo.m
- The parameters for running the code have already been initialized and concern the rotation speed, motor synchronization, arm calibration and alpha1 parameters and alpha2.
- Run main_demo. This code create 16 ply files corresponding to the 16 bands of scan. These files can then ben merged.
- The optimisation parameters can be checked with the function calibration_alpha1.m and calibration_alpha2.m called by the calibration_demo.m file.
