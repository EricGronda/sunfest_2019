%-------------------------------------------------------------------------
% mirror_pts() generates the intersection points between the mirror
%                    and the fov lines
% Input:       tilt
%              distance
%              px_min
%              px_max
% Output:      p_min; point found based on minimum pixel
%              p_max; point found based on maximum value
function [ p_min , p_max ] = mirror_pts(mirror , cam_min , cam_max)

    % Calculate cross products
    p_min = cross( [1, cam_min , 0] , mirror );
    p_max = cross( [1, cam_max , 0] , mirror );
    
end