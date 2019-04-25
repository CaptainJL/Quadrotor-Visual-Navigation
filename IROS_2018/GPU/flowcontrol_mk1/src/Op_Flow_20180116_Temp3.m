clear all; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                            w
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Parameters
% get size

min_num = 0;
max_num = 1384;
%el = 0; % elevation angle (be in y direction) 
%az = 0; % azimuth (be in x direction)

test = sprintf('%d_x.csv',min_num);
A = csvread(test);

radius = 110; % determine the solid angle 
pixel_length = 1; 
f = size(A,2)/2; % focal length, assume the full planar image plane covers 90 degrees view 
col = size(A,2); % cols and rows on the image plane to determine the pixel location
row = size(A,1); 

% Centre of image plane
x0 = col/2;
y0 = row/2;

% Spherical Cap Surface Area (theoretical)
S_cap_area = 2*pi*1*(1-cos(atan(radius/f)));

%% Initialisation
flow_pf_x = []; % sum of average optical flow in x direction (m/frame)
flow_pf_y = []; % sum of average optical flow in x direction (m/frame)
flow_pf_z = []; % sum of average optical flow in x direction (m/frame)
w_x = [];
w_y = [];
w_z = []; % inertial average optical flow 

%% w parameters
theta_0 = atan(radius/f);
lambda = ((sin(theta_0))^2) / (4-(sin(theta_0))^2);
Lambda_m = (pi*(sin(theta_0))^4)/4 * [1/lambda, 0, 0; 0, 1/lambda,0;0,0,2];


%% Read CSV File
for (filenumber = min_num:1:max_num)
    filename_x =  sprintf('%d_x.csv',filenumber);
    filename_y =  sprintf('%d_y.csv',filenumber);
    A = csvread(filename_x);
    A(:,:,2) = csvread(filename_y);
    
    sum_of_flow = [0;0;0];

    for j = 1:1:col
        for i = 1:1:row
            %% calculate the distance fof a pixel to the centre of image plane 
            %  (where optical axis intersects the image plane)
            x_dis = abs(j-x0);
            y_dis = abs(i-y0);
            distance = sqrt(x_dis^2 + y_dis^2);
            %el = atan(y_dis);
            %az = atan(x_dis);

            % check if a pixel is within the solid angle 
            % if so, project onto the spherical cap retina
            %if (distance <= radius)
            if (78 <= distance <= radius)
                flow_2d = [A(i,j,1);A(i,j,2);0]; % extract from data
                mag = sqrt(distance^2 + f^2); % magnitude in 3D
                scale_factor = (1/mag);
                flow_loc = [j-x0;i-y0;f]; % location of the pixel flow
                eta = flow_loc/mag; % normalised flow_loc, represent location on the spherical retina

                % transformational matrix
                eta_t = eta*eta';
                transformation_matrix = eye(3)-eta_t;
                projected_flow_3d = transformation_matrix*(scale_factor*flow_2d);

                % scale with area-weight
                alpha = atan(distance/f); % angle between norm and pixel 

                delta_x = pixel_length/f; % linear scale to a planar plane, r from focal point
                delta_y = pixel_length/f;

                S_pix_area = (abs(cos(alpha)))^3*delta_x*delta_y;
                weight = S_pix_area/S_cap_area;

                sum_of_flow = sum_of_flow + (weight*projected_flow_3d);
            end
        end
    end
    %% Conversion to m/s
    sum_of_flow = sum_of_flow * 60; % 60Hz frame
    flow_pf_x = [flow_pf_x;sum_of_flow(1)]; % sum of average optical flow in x direction (m/frame)
    flow_pf_y = [flow_pf_y;sum_of_flow(2)]; % sum of average optical flow in y direction (m/frame)
    flow_pf_z = [flow_pf_z;sum_of_flow(3)]; % sum of average optical flow in z direction (m/frame)
    
    %% Compute w
    R_t = eye(3);
    w_flow = -(R_t*(Lambda_m^-1)*R_t')*sum_of_flow;
    w_flow = [cosd(-180),0,-sind(-180);0,1,0;sind(-180),0,cosd(-180)]*w_flow;
    w_x = [w_x;w_flow(1)];
    w_y = [w_y;w_flow(2)];
    w_z = [w_z;w_flow(3)];
end

%% Plotting

% %% Plot w
% 
x_axis = [min_num:1:max_num];

figure;
plot(x_axis,w_y)
ylim([-1.3,1.3])
title('w in Y direction (r = 78 pix)')
%title('w in Y direction (solid)')
xlabel('frame #')
ylabel('m/s')

figure;
plot(x_axis,w_x)
ylim([-1.3,1.3])
title('w in X direction (r = 78 pix)')
%title('w in X direction (solid)')
xlabel('frame #')
ylabel('m/s')

figure;
plot(x_axis,w_z)
ylim([-1.3,1.3])
title('w in Z direction (r = 78 pix)')
%title('w in Z direction (solid)')
xlabel('frame #')
ylabel('m/s')



















