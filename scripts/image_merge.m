%--------------------------------------------------------------------------
%
%                           image_merge.m
%
%   This function merges two images given the homography H between them.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function I = image_merge(img1_rgb, img2_rgb, H, params)
    % Loop over the three image channels
    for i = 1:size(img1_rgb,3)
        % Get the i-th color channel
        img1 = img1_rgb(:,:,i);
        img2 = img2_rgb(:,:,i);
        % Warping the second image to the plane of the first one
        [img2_warp, bb, ~] = imwarp(img2, inv(H), 'linear', 'valid');
        % Eliminate from the warped image NaN values resulted from the warping
        ind = find(isnan(img2_warp)); 
        img2_warp(ind) = 0;
        % Reorder the coordinates of the warped image's bounding box
        bb_ij = [bb(2); bb(1); bb(4); bb(3)];
        % Group the coordinates of the warped and original images' bbs
        corners = [bb_ij [0; 0; size(img1)']];
        % Compute the minimum and maximum coordinates of the two bbs to get the coordinates of the total bb
        bb_mos = [min(corners(1, :)); min(corners(2, :));
            max(corners(3, :)); max(corners(4, :))];
        % Offset of the first image within the mosaic
        offs = [abs(min(corners(1, :))); abs(min(corners(2, :)))];
        sz_mos = bb_mos + [offs; offs];
        % Create the two parts of the mosaicing
        mosaic_ref = zeros(sz_mos(3), sz_mos(4));
        mosaic_mov = mosaic_ref;
        % Place the original image and the warped image into the two parts of the mosaic
        if offs(1) > 0
            if offs(2) > 0
                mosaic_ref(offs(1):(size(img1,1)+offs(1)-1),offs(2):(size(img1,2)+offs(2)-1))=img1;
                mosaic_mov(1:(size(img2_warp,1)),1:size(img2_warp,2))=img2_warp;
            else
                mosaic_ref(offs(1):(size(img1,1)+offs(1)-1),1:size(img1,2))=img1;
                mosaic_mov(1:size(img2_warp,1),abs(bb_ij(2)):(size(img2_warp,2)+abs(bb_ij(2))-1))=img2_warp;
            end
        else
            if offs(2) > 0
                mosaic_ref(1:size(img1,1),offs(2):(size(img1,2)+offs(2)-1))=img1;
                mosaic_mov(bb_ij(1):(size(img2_warp,1)+bb_ij(1)-1),1:size(img2_warp,2))=img2_warp;
            else
                mosaic_ref(1:size(img1,1),1:size(img1,2))=img1;
                mosaic_mov(bb_ij(1):(size(img2_warp,1)+bb_ij(1)-1),bb_ij(2):(size(img2_warp,2)+bb_ij(2)-1))=img2_warp;
            end
        end
        % Compose the images
        tmp = mosaic_ref + mosaic_mov;
        % Get a mask for the overlapping part
        mask_ref = mosaic_ref ~= 0;
        mask_mov = mosaic_mov ~= 0;
        mask = mask_ref & mask_mov;
        % Blend the overlap by taking the weighted average of the images intensities
        tmp(mask) = (1-params.alpha).*mosaic_ref(mask) + params.alpha.*mosaic_mov(mask);
        I(:,:,i) = tmp;
    end
end