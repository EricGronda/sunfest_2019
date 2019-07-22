%-------------------------------------------------------------------
% mirror_line() calculates the equation of the mirror line
% Inputs:   tilt; angle in degrees to tilt mirror
%                 (0 is perpendicular to image plane)
%           distance; distance from image plane to mirror (horizontal)
% Outputs:  mirror_eqn; coefficients of the line equation found
%                       ( ax + by + c = 0 )
function mirror_eqn = mirror_line( tilt , d )
    mirror_eqn = [ 1 , -tand(tilt) , d ];
end