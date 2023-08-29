function [T, A] = DHMatrix(arrays)
% Input:
%   -arrays: a n-vector of vectors composed like this: [alpha a d theta]
% Output:
%   -T: the product of all the matrices corresponding to each vector of arrays
%   -A: 4x4xn tensor containing the homogeneous transformation matrices
%       between consecutive frames

    nums = size(arrays);

    if isa(arrays,'sym')
        T = sym(eye(4));
        A = sym(zeros(4,4,nums(1)));
    else
        T = eye(4);
        A = zeros(4,4,nums(1));
    end
    
    for i = 1:nums(1)
        line = arrays(i, :);
        R = [cos(line(4)) -cos(line(1))*sin(line(4)) sin(line(1))*sin(line(4)) line(2)*cos(line(4));
             sin(line(4)) cos(line(1))*cos(line(4)) -sin(line(1))*cos(line(4)) line(2)*sin(line(4));
             0 sin(line(1)) cos(line(1)) line(3);
             0 0 0 1];
        A(:,:,i) = R;
        T = T * R;   
    end
    
    if isa(T, 'sym')
        T = simplify(T);
    end
end