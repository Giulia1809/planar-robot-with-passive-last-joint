clear;
clc;

% syms t q1(t) q2(t) L1 l2 l1 l3
% 
% syms x(t) y(t) theta3(t)
% 
% xa = l1 * cos(q1);
% ya = l1 * sin(q1);
% 
% xb = L1 * cos(q1) + l2 * cos(q1 + q2);
% yb = L1 * sin(q1) + l2 * sin(q1 + q2);
% 
% xc = x + l3 * cos(theta3);
% yc = y + l3 * sin(theta3);
% 
% simplify(diff(xa,t))
% simplify(diff(ya,t))
% 
% simplify(diff(xb,t))
% simplify(diff(yb,t))
% 
% simplify(diff(xc,t))
% simplify(diff(yc,t))

syms t ml1 ml2 ml3 I1 I2 I3 l1 l2 l3 L1 L2 L3 x(t) y(t) x_d(t) y_d(t) theta3(t) theta3_d(t) psi
%x = sym('x', 'real');
%y = sym('y', 'real');
t = sym('t', 'positive');
ml1 = sym('ml1', 'positive');
ml2 = sym('ml2', 'positive');
ml3 = sym('ml3', 'positive');
I1 = sym('I1', 'positive');
I2 = sym('I2', 'positive');
I3 = sym('I3', 'positive');
l1 = sym('l1', 'positive');
l2 = sym('l2', 'positive');
l3 = sym('l3', 'positive');
L1 = sym('L1', 'positive');
L2 = sym('L2', 'positive');
L3 = sym('L3', 'positive');

c2 = (x^2+y^2-L1^2-L2^2)/(2*L1*L2);
s2 = sqrt(1 - c2^2);

q2 = atan(s2 / c2);
q1 = atan(y/x) - atan(L2*s2/L1+L2*c2);

% simplify(diff(q1,t))
% simplify(diff(q2,t))

q2_d = ((2*x(t)*x_d + 2*y(t)*y_d)/(2*L1*L2*(1 - (- L1^2 - L2^2 + x(t)^2 + y(t)^2)^2/(4*L1^2*L2^2))^(1/2)) + (2*L1*L2*(2*x(t)*x_d + 2*y(t)*y_d)*(1 - (- L1^2 - L2^2 + x(t)^2 + y(t)^2)^2/(4*L1^2*L2^2))^(1/2))/(- L1^2 - L2^2 + x(t)^2 + y(t)^2)^2)/((4*L1^2*L2^2*((- L1^2 - L2^2 + x(t)^2 + y(t)^2)^2/(4*L1^2*L2^2) - 1))/(- L1^2 - L2^2 + x(t)^2 + y(t)^2)^2 - 1);
q1_d = (x(t)*y_d - y(t)*x_d)/(x(t)^2 + y(t)^2) - ((x(t)*x_d + y(t)*y_d)/L1 - ((2*x(t)*x_d + 2*y(t)*y_d)*(- L1^2 - L2^2 + x(t)^2 + y(t)^2))/(4*L1^3*L2*(1 - (- L1^2 - L2^2 + x(t)^2 + y(t)^2)^2/(4*L1^2*L2^2))^(1/2)))/(((- L1^2 - L2^2 + x(t)^2 + y(t)^2)/(2*L1) + (L2*(1 - (- L1^2 - L2^2 + x(t)^2 + y(t)^2)^2/(4*L1^2*L2^2))^(1/2))/L1)^2 + 1);

xa = simplify(l1*cos(q1));
ya = simplify(l1*sin(q1));
xa_d = simplify(-l1*sin(q1)*q1_d);
ya_d = simplify(l1*cos(q1)*q1_d);

xb = simplify(L1*cos(q1) + l2*cos(q1 + q2));
yb = simplify(L1*sin(q1) + l2*sin(q1 + q2));
xb_d = simplify(-l2*sin(q1+q2)*(q1_d+q2_d)-L1*sin(q1)*q1_d);
yb_d = simplify(l2*cos(q1+q2)*(q1_d+q2_d)+L1*cos(q1)*q1_d);

xc = x + l3*cos(theta3);
yc = y + l3*sin(theta3);
xc_d = x_d - l3*sin(theta3)*theta3_d;
yc_d = y_d + l3*cos(theta3)*theta3_d;

%questo simplify potrebbe essere lento
display("calcolo energia cinetica");
T = 1/2*ml1*(xa_d^2 + ya_d^2) + 1/2*I1*q1_d^2 ...
             + 1/2*ml2*(xb_d^2 + yb_d^2) + 1/2*I2*(q1_d + q2_d)^2 ...
             + 1/2*ml3*(xc_d^2 + yc_d^2) + 1/2*I3*theta3_d^2;

g0 = [0;9.81 * cos(psi)];  
display("calcolo energia potenziale");
U = simplify(-ml1*g0'*[xa;ya]-ml2*g0'*[xb;yb]-ml3*g0'*[xc;yc]);
L = T-U;

q = [x, y, theta3_d];
q_d = [x_d, y_d, theta3_d];
display("calcolo derivate della lagrangiana");
D_L_d_q = functionalDerivative(L, q);
D_L_d_qd = functionalDerivative(L, q_d);
D_L_d_t = diff(D_L_d_qd, t);
display("FINE");
EL_eq=simplify(D_L_d_t - D_L_d_q);
display(EL_eq);
