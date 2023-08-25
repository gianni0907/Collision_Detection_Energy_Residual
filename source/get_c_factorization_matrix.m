function C = get_c_factorization_matrix(q2,q3,dq1,dq2,dq3)
%GET_C_FACTORIZATION_MATRIX
%    C = GET_C_FACTORIZATION_MATRIX(Q2,Q3,DQ1,DQ2,DQ3)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    22-Aug-2023 12:21:12

t2 = sin(q3);
t3 = q2.*2.0;
t4 = q3.*2.0;
t5 = sin(t3);
t6 = q3+t3;
t8 = t2.*1.2e+2;
t9 = t3+t4;
t7 = sin(t6);
t10 = sin(t9);
t11 = t5.*4.94e+2;
t12 = t7.*1.2e+2;
t13 = t7.*2.4e+2;
t14 = t10.*6.1e+1;
t15 = t8+t12+t14;
t16 = t11+t13+t14;
t17 = (dq1.*t15)./4.8e+2;
t18 = (dq1.*t16)./4.8e+2;
C = reshape([dq3.*t2.*(-1.0./4.0)-dq2.*t5.*(2.47e+2./2.4e+2)-(dq2.*t7)./2.0-(dq3.*t7)./4.0-dq2.*t10.*(6.1e+1./4.8e+2)-dq3.*t10.*(6.1e+1./4.8e+2),t18,t17,-t18,dq3.*t2.*(-1.0./2.0),(dq2.*t2)./2.0,-t17,t2.*(dq2+dq3).*(-1.0./2.0),0.0],[3,3]);
