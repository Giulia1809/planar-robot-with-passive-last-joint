clear;
clc;
syms t th1(t) th2(t) th3(t) th1_d(t) th2_d(t) th3_d(t) m1 m2 m3 l1 l2 l3 L1 L2 L3 I1 I2 I3 psi
t = sym('t', 'positive');
m1 = sym('m1', 'positive');
m2 = sym('m2', 'positive');
m3 = sym('m3', 'positive');
I1 = sym('I1', 'positive');
I2 = sym('I2', 'positive');
I3 = sym('I3', 'positive');
l1 = sym('l1', 'positive');
l2 = sym('l2', 'positive');
l3 = sym('l3', 'positive');
L1 = sym('L1', 'positive');
L2 = sym('L2', 'positive');
L3 = sym('L3', 'positive');
psi = sym('psi', 'real');

xa = (l1*cos(th1));
ya = (l1*sin(th1));
xa_d = (-l1*sin(th1)*th1_d);
ya_d = (l1*cos(th1)*th1_d);

xb = (L1*cos(th1) + l2*cos(th1 + th2));
yb = (L1*sin(th1) + l2*sin(th1 + th2));
xb_d = (-l2*sin(th1+th2)*(th1_d+th2_d)-L1*sin(th1)*th1_d);
yb_d = (l2*cos(th1+th2)*(th1_d+th2_d)+L1*cos(th1)*th1_d);

xc = (L1*cos(th1) + L2*cos(th1 + th2) + l3*cos(th1+th2+th3));
yc = (L1*sin(th1) + L2*sin(th1 + th2) + l3*sin(th1+th2+th3));
xc_d = (-l3*sin(th1+th2+th3)*(th1_d+th2_d+th3_d)-L2*sin(th1+th2)*(th1_d+th2_d)-L1*sin(th1)*th1_d);
yc_d = (-l3*cos(th1+th2+th3)*(th1_d+th2_d+th3_d)-L2*cos(th1+th2)*(th1_d+th2_d)-L1*cos(th1)*th1_d);;


g0 = [0;9.81 * cos(psi)];

T = simplify(1/2*m1*(xa_d^2 + ya_d^2) + 1/2*I1*th1_d^2 ...
    + 1/2*m2*(xb_d^2 + yb_d^2) + 1/2*I2*(th1_d + th2_d)^2 ...
    + 1/2*m3*(xc_d^2 + yc_d^2) + 1/2*I3*(th1_d + th2_d + th3_d)^2);

U = simplify(-m1*g0'*[xa;ya]-m2*g0'*[xb;yb]-m3*g0'*[xc;yc]);

L = T-U;

q = [th1, th2, th3];
q_d = [th1_d, th2_d, th3_d];
display("calcolo derivate della lagrangiana");
D_L_d_q = functionalDerivative(L, q);
D_L_d_qd = functionalDerivative(L, q_d);
D_L_d_t = diff(D_L_d_qd, t);
display("FINE");
EL_eq=simplify(D_L_d_t - D_L_d_q);
display(EL_eq);