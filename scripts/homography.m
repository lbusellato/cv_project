%--------------------------------------------------------------------------
%
%                           homography.m
%
%   This function computes the homography between two sets of points.
%
%   Author: Lorenzo Busellato, 2023
%
%--------------------------------------------------------------------------
function H = homography(p1,p2)
    A=[];
    for i = 1:size(p1,2)
        P1 = [p1(:,i); 1]; % Homogeneous coordinates of p1
        P2 = [p2(:,i); 1]; % Homogeneous coordinates of p2
        Sz = [  0  -P2(3)  P2(2); 
              P2(3)    0   -P2(1); ...
             -P2(2)  P2(1)     0]; % Skew symmetric matrix of p2   
        kro = kron(P1', Sz);
        A=[A; kro(1,:); kro(2,:)];
    end
    % Solve A*vec(H) = 0 using Singular Value Decomposition
    [u, d, v] = svd(A);
    % The homography is in the last column of v
    H = v(:,end);
    H = reshape(H,3,3);
    H = H./H(3,3);
end