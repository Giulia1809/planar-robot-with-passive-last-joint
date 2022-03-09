function v = controller(y_4_d_des, error, F)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    % error has to be [3d, 2d, d, p]
    v = y_4_d_des + F' * error;
end

