%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate Mount Parameters (DAVIS 346)
%   (units in mm and degrees)

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Specified Parameters
f = 6;      % focal length (mm)
d = 10000;    % distance from image plane to center mirror
tilt = 45;  % angle of tilt in degrees
v = 300;   % distance from center mirror to stereo point in degrees

h_fov = 56.2;   % horizontal field of view in degrees
v_fov = 43.7;   % vertical field of view in degrees
ix = 6.4;       % image width in mm (346px)
iy = 4.81;      % image height in mm (240px)

mirror_thickness = 3;   % mirror thickness in mm

k = [ 326.066 , 0       , 166.814;    % intrinsic matrix for DAVIS 346
      0       , 325.975 , 133.972;    % w/ 6mm lens
      0       , 0       , 1        ];

fx = 326.066; % Intrinsic values under different names
fy = 325.975; % because it's easier to type
px = 166.814; %
py = 133.972; %

cam_x = 346; % Number of pixels in camera's dimensions
cam_y = 260; %

mir_angle = 20; % fixed angle between mirrors

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% CENTER MIRROR DIMENSIONS

% -- Calculate height of center mirror

% Project bounds from image to camera
[ y_min , y_max ] = img2cam( 0 , cam_y , fy , py );

% Now to find points of intersection, p_top, p_bot
z_top = (d+f) / (y_min + 1);
y_top = z_top * y_min;

z_bot = (d+f) / (y_max + 1);
y_bot = z_bot * y_max;

p_top = [ y_top ; z_top ];
p_bot = [ y_bot ; z_bot ];

% Calculate the distance between them for the mirror height in mm
% Pythagorean's
hc = sqrt( (z_top - z_bot) ^ 2 + (y_top - y_bot) ^ 2 );

% -- Calculate the width of the center mirror from the midpoint

% project points to camera plane ( center 1/3 )
[ x_min , x_max ] = img2cam( cam_x / 3 , 2 * cam_x / 3 , fx , px );

% calculate pts of intersection
z_mir = (z_top + z_bot) / 2;
x_left = z_mir * x_min;
x_right = z_mir * x_max;

% width is just subtraction
wc = x_right - x_left;

% Grouping params
center_mirror_dim = [ hc , wc ];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% SIDE MIRROR WIDTH     % AND ANGLE
side_mirror_width = (2 * (v - (wc/2)) * tand(mir_angle)) / ...
     (sind(mir_angle) + cosd(mir_angle) * tand(mir_angle));

% check to see if mirror fits in fov
ws_x = side_mirror_width * cosd( mir_angle );
[ x_min , x_max ] = img2cam( 0 , cam_x / 3 , fx , px );
x_left = z_mir * x_min;
x_right = z_mir * x_max;

% Display messages
fprintf("--- Test 1 --------------------------------------------------\n")
fprintf("Center Mirror Dimensions (h , w): %.2fmm , %.2fmm\n" , hc , wc)
fprintf("Side Width: %.2fmm\n" , side_mirror_width)

if ws_x > x_right - x_left
   fprintf( "Side mirror x val (%.2fmm) out of field of view\n" , ws_x )
else
   fprintf( "Side mirror x val (%.2fmm) within field of view\n" , ws_x )
end

fprintf("\n")

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Last one gave dimesions of mirror based on distance and viewpoint
%
% Now we want to find a viewpoint given that the width of the center mirror
% must equal the width of the side mirror in the x direction and a fixed
% mirror angle

% Side mirror x-value
side_width = center_mirror_dim(2) / cosd(mir_angle);

% Calculate viewpoint
v2 = ( 3 * side_width * cosd(mir_angle) ) / 2;

fprintf("--- Test 2 --------------------------------------------------\n")
fprintf("Side Width based on center: %.2fmm\n" , side_width)
fprintf("Viewpoint based on side with: %.2fmm\n" , v2)
fprintf("Ratio distance / viewpoint: %.2f\n" , d / v2 )
fprintf("\n")
