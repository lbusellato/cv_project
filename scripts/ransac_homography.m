%--------------------------------------------------------------------------
%
%                         ransac_homography.m
%
%   This function applies RANSAC to the computation of the homography
%   between two sets of feature points.
%   It accepts as argument the sets of features feat1 and feat2, the vector
%   of matches between them and a struct of parameters defined as follows:
%
%   params: ransac_thresh       distance threshold for RANSAC
%           ransac_iter         max iterations for RANSAC
%           pixel_tol           pixel tolerance for homography computation
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function H = ransac_homography(feat1, feat2, matches, params)
    outliers_min = inf;
    for i = 1:params.ransac_iter
        % Pick 4 random features and correspondences
        r1 = ceil(rand*length(matches));
        r2 = ceil(rand*length(matches));
        r3 = ceil(rand*length(matches));
        r4 = ceil(rand*length(matches));
        p1 = [feat1(1:2, matches(1,r1))'
              feat1(1:2, matches(1,r2))'
              feat1(1:2, matches(1,r3))'
              feat1(1:2, matches(1,r4))'];
        p2 = [feat2(1:2, matches(2,r1))'
              feat2(1:2, matches(2,r2))'
              feat2(1:2, matches(2,r3))'
              feat2(1:2, matches(2,r4))'];
        % Compute the homography between them
        curr_H = homography(p1', p2');
        % Count the outliers
        outlier_cnt = 0;
        p1 = feat1(1:2, matches(1,:));
        p1 = [p1; ones(1,size(p1,2))]; % p1 in homogeneous coordinates
        p2 = curr_H * p1; % Project p2 with the homography
        p2 = ceil(p2(1:2,:)./p2(3,:));
        % Count how many outliers we get with this homographt
        dist = abs(p2(1:2,:) - feat2(1:2,matches(2,:)));
        dist = dist > params.pixel_tolerance;
        outlier_cnt = sum(all(dist == [1 1]'));
        % If the homography produces less outliers, save it as the best so far
        if outlier_cnt < outliers_min
            H = curr_H;
            outliers_min = outlier_cnt;
        end
        if outliers_min == 0 % Can't improve more than that
            break;
        end
    end
end