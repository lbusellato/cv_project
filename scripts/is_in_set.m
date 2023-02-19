%--------------------------------------------------------------------------
%
%                         is_in_set.m
%
%   This function checks if a given point belongs to a set within a given
%   tolerance.
%
%   Author: Lorenzo Busellato, 2023
%
%--------------------------------------------------------------------------
function ret = is_in_set(point, set, tolerance, index)
    ret = false;
    if(isempty(set))
        return;
    end
    if(abs(point(1,1) - set(index,1)) < tolerance) && ...
      (abs(point(1,2) - set(index,2)) < tolerance)
        ret = true;
        return;
    end
end