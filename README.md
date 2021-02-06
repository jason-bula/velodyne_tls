# Velodyne_tls

This  study  develops  a  method  to  acquire  densepoint  clouds  with  a  low-cost  Velodyne  VLP-16  lidar  system, without using expensive GNSS positioning or IMU. Our setting consists  in  mounting  the  lidar  on  a  motor  to  continuously change the scan direction, which leads to a significant increasein  the  point  cloud  density.  A  post-treatment  reconstructs  the position of each point accounting for the motor angle at the timeof acquisition, and a calibration step accounts for inaccuracies in  the  hardware  assemblage.  The  system  is  tested  in  indoors settings  such  as  buildings  and  abandoned  mines,  but  is  also expected  to  give  good  results  outdoors. 

The research associated with this work is available here : https://doi.org/10.5194/gi-9-385-2020, 2020
For any question please contact me on : jason.bula@unil.ch



Instructions to run the codes

- First, download the example point cloud at this link: https://e.pcloud.link/publink/show?code=XZVD17ZUQLyMjOcDEHMXgtKtdvjNSA4E3pX
- The .pcap file must be in the same folder as main_demo.m
- The parameters for running the code have already been initialized and concern the rotation speed, motor synchronization, arm calibration and alpha1 parameters and alpha2.
- Run main_demo. This code create 16 ply files corresponding to the 16 bands of scan. These files can then ben merged.
- The optimisation parameters can be checked with the function calibration_alpha1.m and calibration_alpha2.m called by the calibration_demo.m file.

==================================================================================================================

Instruction supplements for the use of the system in the field

Assembly of the system
1) Install the motor on the tripod
2) Place the slip ring and its metal rod on the motor.
3) Screw the lidar to the metal elbow. Caution: the lidar cable must point exactly against the bottom of the lidar. 
4) Connect the connections between the slip ring, lidar and power supply.
5) Connect the Ethernet cable to the laptop computer

Notes: By tightening the screws correctly, the backlash between the system components should be avoided as much as possible. The tripod does not necessarily need to be level, but it is preferable. 

Software :
1) Install the "Syrp Genie mini" application on a smartphone (tested on v2.7.6)
2) Install veloview (software present on the USB key provided with the lidar)
3) Download the demo file here : https://github.com/jason-bula/velodyne_tls/blob/master/demo_main.m and open it with Matlab

Example of configuration :
The acquisition time may vary, the slower the motor rotation, the better the resolution. Scans lasting between 3 and 6 minutes are preferable. To avoid measuring the acceleration and deceleration of the motor, the rotation is done on more than 360 degrees and is started before the acquisition. 
Configuration for a rotation of 3,4 and 6 minutes

Angle de rotation (degrés)	Temps d’acquisition (s)	Temps de rotation pour 360 degrés (s)	Vitesse de rotation : 1 degré / seconde
370	370 	360	1
375	250	240	1.5
380	190	180	2

