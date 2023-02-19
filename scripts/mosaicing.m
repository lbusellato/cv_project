%--------------------------------------------------------------------------
%
%                           mosaicing.m
%
%   This function performs image mosaicing on a set of source images.
%   It accepts as argument a struct of parameters defined as follows:
%
%   params: set                 # of the image set
%           peak_thresh         peak threshold for SIFT
%           edge_thresh         edge threshold for SIFT
%           ransac_thresh       distance threshold for RANSAC
%           ransac_iter         max iterations for RANSAC
%           pixel_tol           pixel tolerance for homography computation
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function mosaic = mosaicing(params)
    % Recover the image set's path
    set = dir(strcat("images/set", num2str(params.set), "/"));
    % Load the first image
    img1_rgb = imread(strjoin({set(3).folder,set(3).name},'/'));
    img1 = single(im2gray(img1_rgb));
    mosaic = img1_rgb;
    % Iterate over all other images in the set
    for i = 4:size(set, 1)
        % Adaptive RANSAC threshold, useful when using multiple images
        params.ransac_thresh = i * 2 * params.ransac_thresh;
        % Load the next image
        img2_rgb = imread(strjoin({set(i).folder,set(i).name},'/'));
        img2 = single(im2gray(img2_rgb));
        % Compute both images features and descriptors
        [feat1, desc1] = vl_sift(single(im2gray(uint8(mosaic))));
        [feat2, desc2] = vl_sift(img2);
        % Match the descriptors
        [matches, scores] = vl_ubcmatch(desc1, desc2);
        % Apply RANSAC to the y coordinates
        points = [feat1(2, matches(1,:)); feat2(2, matches(2,:))];
        [ids] = ransac_points(points, params);
        matches = matches(:,ids); 
        % Apply RANSAC to the x coordinates
        points = [feat1(1, matches(1,:)); feat2(1, matches(2,:))];
        [ids] = ransac_points(points, params);
        matches = matches(:,ids); % Final set of matching features
        % Robust homography computation with RANSAC
        H = ransac_homography(feat1, feat2, matches, params);
        % Merge the images
        mosaic = image_merge(mosaic, img2_rgb, H);
    end
    mosaic = uint8(mosaic);
    % Save the result
    path = strcat("mosaics/set", num2str(params.set), "/mosaic.jpg");
    imwrite(uint8(mosaic), path, 'JPEG');
end