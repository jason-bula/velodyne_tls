# Velodyne_tls

This  study  develops  a  method  to  acquire  densepoint  clouds  with  a  low-cost  Velodyne  VLP-16  lidar  system, without using expensive GNSS positioning or IMU. Our setting consists  in  mounting  the  lidar  on  a  motor  to  continuously change the scan direction, which leads to a significant increasein  the  point  cloud  density.  A  post-treatment  reconstructs  the position of each point accounting for the motor angle at the timeof acquisition, and a calibration step accounts for inaccuracies in  the  hardware  assemblage.  The  system  is  tested  in  indoors settings  such  as  buildings  and  abandoned  mines,  but  is  also expected  to  give  good  results  outdoors. 

The research associated with this work is available here : https://doi.org/10.5194/gi-9-385-2020, 2020




Instructions to run the codes

- First, download the example point cloud at this link: https://www96.zippyshare.com/v/DoaqH8zs/file.html
- The .pcap file must be in the same folder as main_demo.m
- The parameters for running the code have already been initialized and concern the rotation speed, motor synchronization, arm calibration and alpha1 parameters and alpha2.
- Run main_demo. This code create 16 ply files corresponding to the 16 bands of scan. These files can then ben merged.
- The optimisation parameters can be checked with the function calibration_alpha1.m and calibration_alpha2.m called by the calibration_demo.m file.
