%--------------------------------------------------------------------------
%
%                         ransac_points.m
%
%   This function applies RANSAC to a set of feature points coordinates.
%   It accepts as argument a matrix of feature coordinates and a struct of 
%   parameters defined as follows:
%
%   params: ransac_thresh       distance threshold for RANSAC
%           ransac_iter         max iterations for RANSAC
%
%   The function returns a vector of the indexes of the detected inliers.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function [ids] = ransac_points(points, params)
    % Normalize data
    p1 = points(1,:); p2 = points(2,:);
    p1 = p1./max(p1); p2 = p2./max(p2);
    points = [p1; p2]';
    % RANSAC
    outliers_min = inf;
    for i = 1:params.ransac_iter
        % Reset variables
        outlier_cnt = 0; % This iteration's # of outliers
        curr_ids = []; % This iteration's inliers indexes
        % Randomly select 2 points
        random_sel = ceil(rand(1,2)*size(points,1));
        candidate_inliers = points(random_sel(:),:);
        % Fit a line to those points
        x1 = candidate_inliers(1,1); x2 = candidate_inliers(2,1); 
        y1 = candidate_inliers(1,2); y2 = candidate_inliers(2,2); 
        m = (y1 - y2) / (x1 - x2);
        q = y1 - m*x1;
        % Use the line as a model
        y = m*points(:,1) + q;
        dist = abs(points(:,2) - y);
        % Binary mask of the inlier ids
        dist = dist < params.ransac_thresh;
        % Count the outliers
        outlier_cnt = sum(dist==0);
        if outlier_cnt < outliers_min
            outliers_min = outlier_cnt;
            ids = dist;
        end
    end
end