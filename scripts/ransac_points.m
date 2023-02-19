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
    max_inlier_cnt = 0;
    ids = [];
    f = waitbar(0, 'RANSAC - features'); % Progress bar
    for i = 1:params.ransac_iter
        waitbar(i/params.ransac_iter, f, sprintf('RANSAC - features: %d %%', floor(i/params.ransac_iter*100)));
        % Reset variables
        inlier_cnt = 0; % This iteration's # of inliers
        curr_ids = []; % This iteration's inliers indexes
        % Randomly select 2 points
        random_sel = ceil(rand(1,2)*size(points,1));
        candidate_inliers = points(random_sel(:),:);
        % Fit a line to those points
        x1 = candidate_inliers(1,1); x2 = candidate_inliers(2,1); 
        y1 = candidate_inliers(1,2); y2 = candidate_inliers(2,2); 
        m = (y1 - y2) / (x1 - x2);
        q = y1 - m*x1;
        % Compute inliers
        for id = 1:size(points,1)
            y = points(id,1)*m + q;
            dist = abs(points(id,2) - y)/sqrt(1 + m^2);
            if dist < params.ransac_thresh % Inlier detected
                inlier_cnt = inlier_cnt + 1;
                curr_ids(inlier_cnt) = id;
            end
        end
        if inlier_cnt > max_inlier_cnt % This iteration produces more inliers
            max_inlier_cnt = inlier_cnt;
            ids = curr_ids;
        end
    end
    close(f);
end