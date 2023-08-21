function q = inverse_kinematics(p,l,soln_type)
% compute inverse kinematics of 3R spatial manipulator
% from Cartesian coordinates of ee position to joint configuration
% input: Cartesian coordinates p = [x; y; z]
%        link lengths l = [l1; l2; l3]
%        specified solution among the 4 possibilities
%        soln_type is a string containing one of the following
%        1. 'pp' for positive-positive
%        2. 'pn' for positive-negative
%        3. 'np' for negative-positive
%        4. 'nn' for negative-negative
% output: joint configuration q = [q1; q2; q3]
arguments
    p (:,1) double
    l (:,1) double
    soln_type char = 'nn'
end
if (~strcmp(soln_type,'pp') && ~strcmp(soln_type,'pn') ...
    && ~strcmp(soln_type,'np') && ~strcmp(soln_type,'nn'))
    error("Specified solution type not valid");
end
    
c3 = (p(1)^2+p(2)^2+(p(3)-l(1))^2-l(2)^2-l(3)^2)/(2*l(2)*l(3));

if (soln_type(2)=='p')
    s3 = sqrt(1-c3^2);
else
    s3 = -sqrt(1-c3^2);
end
q3=atan2(s3,c3);

% compute q1 (positive or negative)
if (p(1)^2+p(2)^2 < 10^(-10))
    q1 = 0;
elseif (soln_type(1)=='p')
    q1=atan2(p(2),p(1));
else
    q1=atan2(-p(2),-p(1));
end

% det matrix
detM=p(1)^2+p(2)^2+(p(3)-l(1))^2;
if (detM==0)
    q2 = 0;
else
    % q2 depends of sign of q1 and q3
    c2=((l(2)+l(3)*c3)*(p(1)*cos(q1)+p(2)*sin(q1))+(l(3)*s3)*(p(3)-l(1)))/detM;
    s2=((-l(3)*s3)*(p(1)*cos(q1)+p(2)*sin(q1))+(l(2)+l(3)*c3)*(p(3)-l(1)))/detM;
    q2=atan2(s2,c2);
end
q = [q1; q2; q3];
end