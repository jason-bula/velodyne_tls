%%   Dense  point  cloud  acquisition  with  a  low-cost  Velodyne  VLP-16
%    Author : Jason Bula
%    Under the supervision of : Grégoire Mariéthoz
%    University of Lausanne

%% Directory Management
clear, close all

% Source files folder (.pcap)
input = 'C:\Users\jason\Desktop\Dense  point  cloud  acquisition  with  a  low-cost  Velodyne  VLP-16';
% Result folder
mkdir result
output ='result\';

% input file name
input_file_name = 'baulmes_demo.pcap';

% output file name
output_file_name = 'calibration_';

cd(input)

%% Initialization of parameters 

times = 1800; % Scan duration in tenths of a second (sec^-1) 
angle = 360; % LiDAR rotation angle
first = 199; % Time at the first frame

%% Export parameters
% Set to 0 to keep all the points
% Set to 1 to keep the x-positives values
% Set to 2 to keep the y-negatives values
pos = 1;


% Set to 0 to keep save the band 1 and 16
% Set to 1 to keep all the values
% Set to 2 to keep only band 8

pos2 = 0;

%Set the grid resolution in [m] 
%Set to 0 to keep all the values
gridStep = 0; 

%% Calibration parameters
% Correction angle alpha_1 and alpha_2 [degrees]
% Those angle are determined automatically after calibration or be chosen

alpha_1 = 0.01;
alpha_2 = [-0.735500000000001]; %initial 0.5
%alpha_3 = 1;
% Arm length [m]
R =     -0.0   ;
theta3 = -0.0; %initial -055



%% Point cloud correction to be applied during rotation
% 
%  In this part of the code, the scan will be extracted frame by frame 
%  in order to to apply the necessary correction to realign the point cloud.
%  First of all, only images containing positive X will be kept, then each
%  image will be processed separately.
%  Two transformation matrices will be applied to each image. One
%  containing rotation according to the LiDAR angle (varies over time) and
%  the other, an applied translation corresponding to the distance between 
%  the arm and the optical center of the LiDAR. This distance was measured
%  manually and corresponds to a displacement over z of R = 0.0945m (can
%  be improved). Finally, each image is saved separately in a Cell.
% 

% Initialisation
veloReader = velodyneFileReader(input_file_name,'VLP16'); 
last = first +times; % Time at the last frame
angle_deg=(0:angle/times:angle); % Angle after each frame
angle = deg2rad(angle_deg); % Angle in radian
s = 0; 

for i = 2 : length(angle) % Runs as many times as there are frames
     NF = first + s;
     ptCloudIn = readFrame(veloReader,NF); % Selecting frames separately

% Apply export parameters  
if pos == 1   
    
    ptCloudIn3 =  ptCloudIn.Location(:,:,1);
    ptCloudIn3(isnan(ptCloudIn3))=0;
    ptCloudIn3(ptCloudIn3<0)=0;
    ptCloudIn3(ptCloudIn3>0)=1;
    ptCloudIn = pointCloud(ptCloudIn.Location(:,:,:).*ptCloudIn3);
end

if pos == 2   
    
    ptCloudIn3 =  ptCloudIn.Location(:,:,1);
    ptCloudIn3(isnan(ptCloudIn3))=0;
    ptCloudIn3(ptCloudIn3>0)=0;
    ptCloudIn3(ptCloudIn3<0)=1;
    ptCloudIn = pointCloud(ptCloudIn.Location(:,:,:).*ptCloudIn3);
end
  
clear ptCloudIn2 ptCloudIn3 

% Transformation of each image
% Définition des matrices de transformation

% Alignment correction as a function of motor speed (y-axis)
VM = [cos(angle(i)) 0 sin(angle(i)) 0; 0 1 0 0; -sin( angle(i)) 0 ...
      cos(angle(i)) 0; 0 0 0 1];

  % Alpha_1 correction (x-axis)
A1 = [1 0 0 0; 0 cosd(alpha_1) sind(alpha_1) 0 ; ...
      0 -sind(alpha_1) cosd(alpha_1) 0 ; 0 0 0 1];  
  
% Alpha_2 correction (z-axis)
A2 =  [cosd(alpha_2) sind(alpha_2) 0 0; ...
     -sind(alpha_2) cosd(alpha_2) 0 0; 0 0 1 0; 0 0 0 1];
 
%A3 = [cosd(alpha_3) 0 sind(alpha_3) 0; 0 1 0 0; -sind(alpha_3) 0 ...
%      cosd(alpha_3) 0; 0 0 0 1]; 
  
% R correction
T = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 R 1];

% Speed adjustement
T4 = [cosd(theta3) sind(theta3) 0 0; ...
     -sind(theta3) cosd(theta3) 0 0; 0 0 1 0; 0 0 0 1];
 
% Apply transformation
tform_T = affine3d(T);
ptCloudIn = pctransform(ptCloudIn,tform_T);

tform_A1 = affine3d(A1);
ptCloudIn = pctransform(ptCloudIn,tform_A1);

tform_A2 = affine3d(A2);
ptCloudIn = pctransform(ptCloudIn,tform_A2);

%tform_A3 = affine3d(A3);
%ptCloudIn = pctransform(ptCloudIn,tform_A3);

tform_T = affine3d(T4);
curr_img = pctransform(ptCloudIn,tform_T); % configuration 2

tform_R = affine3d(VM);
Nuage{i-1} = pctransform(curr_img,tform_R); % Save in a cell

s = s + 1; % count update

end


% Initialisation of export parameters
if pos2 == 1
    bandNumber = 16;
else
    bandNumber = 2;
end

if pos2 == 2
    bandNumber = 1;
end

%% Point cloud merging 

for bande_sep = 1 : bandNumber
    
if pos2 == 1 
    iiii=bande_sep;
end

if pos2 == 0    
    Bande_calibration = [1 16];
    iiii = Bande_calibration(bande_sep);
end

if pos2 == 2    
    Bande_calibration = 8;
    iiii = Bande_calibration(bande_sep);
end

% Variable initialization
x = [];
y = [];
z = [];

X_ref = [];
Y_ref = [];
Z_ref = [];

X_ref_final = [];
Y_ref_final = [];
Z_ref_final = [];

% Acceleration of the process by combining several loops
for iii = 1 : 10 
    for ii = (times/10*iii)-((times/10)-1) : iii*times/10     
     for i = iiii:iiii % save the band separately

% Deleting old values
        x1 = [];
        y1 = [];
        z1 = [];
  
% Selection of points in the correct matrix locations
% Point clouds are recorded as follows: 16*1800*3
% 16 corresponds to the band, 1800 corresponds to the number of points recorded
% per band, 3 corresponds to the x, y and z values.

x1(i,:) = Nuage{1,ii}.Location(i,:,1);
y1(i,:) = Nuage{1,ii}.Location(i,:,2);
z1(i,:) = Nuage{1,ii}.Location(i,:,3);


x = [x x1(i,:)];
y = [y y1(i,:)];
z = [z z1(i,:)];



    end 
    X_ref = [X_ref x];
    Y_ref = [Y_ref y];
    Z_ref = [Z_ref z];

    x = 0;
    y = 0;
    z = 0;
    end
    X_ref_final = [X_ref_final X_ref];
    Y_ref_final = [Y_ref_final Y_ref];
    Z_ref_final = [Z_ref_final Z_ref];
    
% disp(iii) % Compteur de progression de l'extraction 
    X_ref = 0;
    Y_ref = 0;
    Z_ref = 0;
end
        

%% Reconstruction of the point cloud
ref = [X_ref_final; Y_ref_final; Z_ref_final]'; 
%ref2{pos} = [X_ref_final; Y_ref_final; Z_ref_final]'; 
%disp('ref')    


PC_corr1 = pointCloud(ref);

if gridStep == 0
    PC_downsampled_1 = PC_corr1;
else
    PC_downsampled_1 = pcdownsample(PC_corr1,'gridAverage',gridStep);
end

% Remove invalid values
[PC_Final1,indices]= removeInvalidPoints(PC_downsampled_1);

%% Point cloud export
% In this part of the code, the scatterplot is exported. 

% Defining output document names
Nom_fichier = sprintf([output_file_name num2str(iiii)]); 

cd(output)
% Write file
pcwrite(PC_Final1,Nom_fichier,'PLYFormat','binary');

cd(input)
%ref = []; % suppression of the loaded point cloud
disp([num2str(iiii) ' of 16'])
end


disp('Densification completed')


