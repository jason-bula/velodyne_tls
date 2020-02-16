%% Hardware Calibration : Point Cloud Densification
%    Author : Jason Bula
%    Under the supervision of : Grégoire Mariéthoz
%    University of Lausanne
%% Directory Management
clear, close all

% Calibration function to call : 1 for alpha1, 2 for alpha2
calibration = 2;

% Source files folder (.pcap)
global input
input = 'C:\Users\jason\Desktop\Dense  point  cloud  acquisition  with  a  low-cost  Velodyne  VLP-16';

% Result folder
global output
output ='C:\Users\jason\Desktop\Dense  point  cloud  acquisition  with  a  low-cost  Velodyne  VLP-16';

% input file name
global input_file_name
input_file_name = 'baulmes_demo.pcap';

% output file name
global output_file_name
output_file_name = 'calib';

%% Initialization of parameters 
global times
times = 1800; % Scan duration in tenths of a second (sec^-1)
global angle
angle = 360; % LiDAR rotation angle
global first
first = 199; % Time at the first frame

% Process Speed : set to 1,2 or 5 (5 is the fastest)
global mult
mult =  2;

% The length of the arm
global R
R = 0.0945;

% the distance from the lidar where the optimization is done
global Range 
Range = [4 30];

global gridStep
gridStep = 0.01;

cd(input)

global veloReader 
veloReader = velodyneFileReader(input_file_name,'VLP16'); 
%theta0 = [0	0];

cd(input);
%% Calibration alpha1
if calibration == 1

options = optimset('Display','iter','PlotFcns',@optimplotfval);

% Initial parameter
theta00 = 3;
fun = @alpha1_calibration;

[x,fval,exitflag,output] = fminsearch(fun,theta00,options)
%cd(output)
end
%% Calibration alpha2
if calibration == 2


options = optimset('Display','iter','PlotFcns',@optimplotfval);

% Initial parameter
theta00 = 0;
fun = @alpha2_calibration;

[x,fval,exitflag,output] = fminsearch(fun,theta00,options)
end