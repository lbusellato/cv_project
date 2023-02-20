%--------------------------------------------------------------------------
%
%                           imwarp.m
%
%   This function merges two images given the homography H between them.
%   Adapted to work with RGB images from the imwarp function by A.Fusiello.   
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function [I2, bb] = imwarp(I,H)
    % Compute the smallest bb containing the whole image
    corners = [0, 0, size(I,2), size(I,2);
               0, size(I,1), 0, size(I,1)];
    corners_x=p2t(H,corners);
    minx = floor(min(corners_x(1,:)));
    maxx = ceil(max(corners_x(1,:)));
    miny = floor(min(corners_x(2,:)));
    maxy = ceil(max(corners_x(2,:)));   
    bb = [minx; miny; maxx; maxy];
    [x,y] = meshgrid(minx:maxx-1,miny:maxy-1);
    pp = p2t(inv(H),[vec(x)';vec(y)']);
    xi=ivec(pp(1,:)',size(x,1));
    yi=ivec(pp(2,:)',size(y,1));
    I2(:,:,1)=interp2(0:size(I(:,:,1),2)-1, 0:size(I(:,:,1),1)-1,double(I(:,:,1)),xi,yi,'linear',NaN);
    I2(:,:,2)=interp2(0:size(I(:,:,2),2)-1, 0:size(I(:,:,2),1)-1,double(I(:,:,2)),xi,yi,'linear',NaN);
    I2(:,:,3)=interp2(0:size(I(:,:,3),2)-1, 0:size(I(:,:,3),1)-1,double(I(:,:,3)),xi,yi,'linear',NaN);
end