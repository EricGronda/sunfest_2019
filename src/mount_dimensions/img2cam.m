%----------------------------------------------------------------------
% img2cam() converts 2 pixels (same dimension) from image plane 
%           to camera plane
% Inputs:   px_min; min pixel number to convert to camera plane
%           px_max; max pixel number to convert to camera plane
%           f;  focal length in specified dimension (intrinsic matrix)
%           p;  principal point offset (intrinsic matrix)
% Outputs:  cam_min; px_min value in camera plane
%           cam_max; px_max value in camera plane
function [cam_min , cam_max] = img2cam( px_min , px_max , f , p )
    cam_min = (px_min - p) / f;
    cam_max = (px_max - p) / f; 
end