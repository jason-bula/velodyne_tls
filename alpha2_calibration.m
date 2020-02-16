%% this function use the Nelder-Mead algorith to optimze the angle alpha2
function [to_compare] = alpha2_calibration(theta00)




%% Directory Management
global input
global output
global input_file_name
global output_file_name
global times
global angle
global first
global Range
global veloReader
global mult
global gridStep
global R
%%


angle_deg=(0:angle/times:angle); % Angle after each frame
angle2 = deg2rad(angle_deg); % Angle in radian




alpha_1 = 0 ;% initial: = 0; alpha_1 and alpha_2 must be optimize separately
alpha_2 = theta00(1); 

cd(input)


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
            for comp = 1 : 2
s = 0;

for i = 1 : (length(angle2)-1)/mult % Runs as many times as there are framesges
    NF = (first + s);
     ptCloudIn = readFrame(veloReader,NF); % Selecting frames separately
    
 %selection of the positive values on the x-axis    
      if comp == 1
    ptCloudIn3 =  ptCloudIn.Location(1,:,1);
    ptCloudIn3(isnan(ptCloudIn3))=0;
    ptCloudIn3(ptCloudIn3<0)=0;
    ptCloudIn3(ptCloudIn3>0)=1;
    ptCloudIn = pointCloud(ptCloudIn.Location(:,:,:).*ptCloudIn3);     

%selection of the negative values on the x-axis    
      else
    ptCloudIn3 =  ptCloudIn.Location(1,:,1);
    ptCloudIn3(isnan(ptCloudIn3))=0;
    ptCloudIn3(ptCloudIn3>0)=0;
    ptCloudIn3(ptCloudIn3<0)=1;
    ptCloudIn = pointCloud(ptCloudIn.Location(:,:,:).*ptCloudIn3);      
      end


clear ptCloudIn2 ptCloudIn3 

% Transformation of each image
% Alignment correction as a function of motor speed
VM = [cos(angle2(s+2)) 0 sin(angle2(s+2)) 0; 0 1 0 0; -sin( angle2(s+2)) 0 ...
      cos(angle2(s+2)) 0; 0 0 0 1];
% Alpha_1 correction
A1 = [1 0 0 0; 0 cosd(alpha_1) sind(alpha_1) 0 ; ...
      0 -sind(alpha_1) cosd(alpha_1) 0 ; 0 0 0 1];  
  
% Alpha_2 correction (z-axis)
A2 =  [cosd(alpha_2) sind(alpha_2) 0 0; ...
     -sind(alpha_2) cosd(alpha_2) 0 0; 0 0 1 0; 0 0 0 1];
 
% R correction
T = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 R 1];

% Apply transformation
tform_T = affine3d(T);
ptCloudIn = pctransform(ptCloudIn,tform_T);

tform_A1 = affine3d(A1);
ptCloudIn = pctransform(ptCloudIn,tform_A1);

tform_A2 = affine3d(A2);
ptCloudIn = pctransform(ptCloudIn,tform_A2);



tform_R = affine3d(VM);
Nuage{i} = pctransform(ptCloudIn,tform_R); % Save in a cell

s = s + mult; 

end


for bande_sep = 1 : 1
%   
iiii=bande_sep;
% cette ligne peut être activée dans le but de
% calibrer la verticale. Le code enregistrera alors seulement les bandes
% 1 et 16 permettant ainsi de les aligner correctement selon le
% paramètre theta choisi
    

% Initialisation des variables

x = [];
y = [];
z = [];

X_ref = [];
Y_ref = [];
Z_ref = [];

X_ref_final = [];
Y_ref_final = [];
Z_ref_final = [];

% Accélération du processus en combinant plusieurs boucles
for iii = 1 : 10/mult 
    for ii = (times/10*iii)-((times/10)-1) : iii*times/10     
     for i = 8:8 %16 car le fichier est organisé en 16 bandes % attention modifications effectuées

%suppression des anciennes valeurs
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
    
    X_ref = 0;
    Y_ref = 0;
    Z_ref = 0;
end
   % X_ref = [X_ref ]
        
disp('Extraction done')

%% Reconstruction of the point cloud
ref = [X_ref_final; Y_ref_final; Z_ref_final]'; 
ref(isnan(ref))=0;
dist = (ref(:,1).^2 + ref(:,2).^2 + ref(:,3).^2).^(1/2);
A = (dist>Range(1) & dist<Range(2)); %apply the comparison range
xplan = (A.*ref(:,1));
yplan = (A.*ref(:,2));
zplan = (A.*ref(:,3));
ref =[xplan yplan zplan];

PC_corr1 = pointCloud(ref);

[PC_Final1,indices]= removeInvalidPoints(PC_corr1);
disp('removeInvalidPoints done')

%% Point cloud export

% Update of the file name
file_name1 = sprintf([output_file_name num2str(iiii)]); 
ref = []; 
extract{comp} = PC_Final1;

end
            end
            
extract1 = pointCloud(extract{1,1}.Location);
extract2 = pointCloud(extract{1,2}.Location);

%downsampling before comparison
extract1 = pcdownsample(extract1,'gridAverage',gridStep);
extract2 = pcdownsample(extract2,'gridAverage',gridStep);

%% Nelder-Mead Optimisation
tform = pcregistericp(extract1,extract2,'Extrapolate',true);
to_compare = abs(sum(tform.T(4,:))-1);


end

