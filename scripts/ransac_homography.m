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
    f = waitbar(0, 'RANSAC - homograpy'); % Progress bar
    for i = 1:params.ransac_iter
        waitbar(i/params.ransac_iter, f, sprintf('RANSAC - homography: %d %%', floor(i/params.ransac_iter*100)));
        % Pick 4 random features
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
        for i = 1:length(matches)
            p1 = feat1(1:2, matches(1,i));
            p1 = [p1(1:2); 1]; % p1 in homogeneous coordinates
            p2 = curr_H * p1; % Project p2 with the homography
            p2 = ceil(p2./p2(end));
            % Update outlier count
            if ~is_in_set(p2', feat2(1:2,matches(2,:))', params.pixel_tolerance, i)
                outlier_cnt = outlier_cnt + 1;
            end
        end
        % If the homography produces less outliers, save it as the best so far
        if outlier_cnt < outliers_min
            H = curr_H;
            outliers_min = outlier_cnt;
        end
        if outliers_min == 0 % Can't improve more than that
            break;
        end
    end
    close(f);
end