%--------------------------------------------------------------------------
%
%                           image_merge.m
%
%   This function merges two images given the homography H between them.
%   The function also performs image blending with 2 available modes:
%       - 'none': no image blending
%       - 'average': averages pixel intensities in the overlapping region
%       - 'linear': applies linear alpha blending
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function I = image_merge(img1_rgb, img2_rgb, H, params)
    % Warping the second image to the plane of the first one
    [img2_warp, bb] = rgb_imwarp(img2_rgb, inv(H));
    % Eliminate from the warped image NaN values resulted from the warping
    ind = find(isnan(img2_warp)); 
    img2_warp(ind) = 0;
    % Reorder the coordinates of the warped image's bounding box
    bb_ij = [bb(2); bb(1); bb(4); bb(3)];
    % Group the coordinates of the warped and original images' bbs
    corners = [bb_ij [0; 0; size(img1_rgb(:,:,1))']];
    % Compute the minimum and maximum coordinates of the two bbs to get the coordinates of the total bb
    bb_mos = [min(corners(1, :)); min(corners(2, :));
        max(corners(3, :)); max(corners(4, :))];
    % Offset of the first image within the mosaic
    offs = [abs(min(corners(1, :))); abs(min(corners(2, :)))];
    sz_mos = bb_mos + [offs; offs];
    % Create the two parts of the mosaicing
    mosaic_ref = zeros(sz_mos(3), sz_mos(4), 3);
    mosaic_mov = mosaic_ref;
    % Place the original image and the warped image into the two parts of the mosaic
    s1 = size(img1_rgb(:,:,1),1);
    s2 = size(img1_rgb(:,:,1),2);
    if offs(1) > 0
        if offs(2) > 0
            mosaic_ref(offs(1):(s1+offs(1)-1),offs(2):(s2+offs(2)-1),:)=img1_rgb(:,:,:);
            mosaic_mov(1:(size(img2_warp,1)),1:size(img2_warp,2),:)=img2_warp(:,:,:);
        else
            mosaic_ref(offs(1):(s1+offs(1)-1),1:s2,:)=img1_rgb(:,:,:);
            mosaic_mov(1:size(img2_warp,1),abs(bb_ij(2)):(size(img2_warp,2)+abs(bb_ij(2))-1),:)=img2_warp(:,:,:);
        end
    else
        if offs(2) > 0
            mosaic_ref(1:s1,offs(2):(s2+offs(2)-1),:)=img1_rgb(:,:,:);
            mosaic_mov(bb_ij(1):(size(img2_warp,1)+bb_ij(1)-1),1:size(img2_warp,2),:)=img2_warp(:,:,:);
        else
            mosaic_ref(1:s1,1:s2,:)=img1_rgb(:,:,:);
            mosaic_mov(bb_ij(1):(size(img2_warp,1)+bb_ij(1)-1),bb_ij(2):(size(img2_warp,2)+bb_ij(2)-1),:)=img2_warp(:,:,:);
        end
    end
    % Compose the images
    I = mosaic_ref + mosaic_mov;
    % Apply blending
    if strcmp(params.blending, 'average')
        % Get a mask for the overlapping part
        mask_ref = mosaic_ref ~= 0;
        mask_mov = mosaic_mov ~= 0;
        mask = mask_ref & mask_mov;
        % Working channel-by-channel introduces some holes in the mask
        mask = imfill(mask,'holes'); 
        % Blend the overlap by taking the weighted average of the images intensities
        I(mask) =0.5*(mosaic_ref(mask) + mosaic_mov(mask));
    elseif strcmp(params.blending, 'linear')
        % Get a mask for the overlapping part
        mask_ref = mosaic_ref ~= 0;
        mask_mov = mosaic_mov ~= 0;
        mask = mask_ref & mask_mov;
        % Working channel-by-channel introduces some holes in the mask
        mask = imfill(mask,'holes'); 
        % Remove the overlap, the blending will take care of it
        I(mask) = 0;
        % Create an alpha mask
        alpha_mask = zeros(size(mask,1),size(mask,2));
        for j = 1:size(mask,1)
            % Find the first and last non-zero columns
            minIdx = find(mask(j,:,1),1);
            maxIdx = find(mask(j,:,1),1,'last');
            if size(minIdx,2) == 1 && size(maxIdx,2) == 1
                % Create a gradient in the overlap
                decrease_step = 1/(maxIdx - minIdx);
                for m = minIdx:maxIdx+1
                    alpha_mask(j,m) = 1 - decrease_step*(m-minIdx);
                end            
            end
        end
        % Extract the parts of the images that overlap
        mr = uint8(double(mask).*mosaic_ref);
        mm = uint8(double(mask).*mosaic_mov);
        % Apply alpha blending
        I = uint8(I) + uint8(alpha_mask.*double(mr) + (1-alpha_mask).*double(mm));
    end
end