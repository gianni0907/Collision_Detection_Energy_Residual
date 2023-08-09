function A = DHMatrix(table)
% inputs:
%  -table: the DH parameters table, a n-vector of vectors composed with 
%  this structure: [alpha a d theta]
% output:
%  -A: cell array with homogeneous relative transformations from RF i-1 to
%   RF i
    nums = size(table);
    
    A = zeros(4,4,nums(1));
    
    for i = 1:nums(1)
        line = table(i, :);
        R = [cos(line(4)) -cos(line(1))*sin(line(4)) sin(line(1))*sin(line(4)) line(2)*cos(line(4));
             sin(line(4)) cos(line(1))*cos(line(4)) -sin(line(1))*cos(line(4)) line(2)*sin(line(4));
             0 sin(line(1)) cos(line(1)) line(3);
             0 0 0 1;];
        A(:,:,i) = R;
    end
   
end