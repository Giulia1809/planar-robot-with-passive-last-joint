function y_i_des = trajectoryGeneration(y_s, y_g, T)
%UNTITLED Summary of this function goes here
%   y_s = [y_s; y_s_d; y_s_d_d; y_s_3_d]
%   y_g = [y_g; y_g_d; y_g_d_d; y_g_3_d]
%   y_i_des = [y_des; y_des_d; y_des_d_d; y_des_3_d; y_des_4_d]

    a0 = y_s(1);
    a1 = y_s(2) * T;
    a2 = 1/2 * y_s(3) * T^2;
    a3 = 1/6 * y_s(4) * T^3;
    a4 = 35 * (y_g(1) - y_s(1)) - (20 * y_s(2) + 15 * y_g(2)) * T - (5 * y_s(3) - 5/2* y_g(3)) * T^2 ...
         -(2/3 * y_s(4) + 1/6 * y_g(4)) * T^3;
    a5 = -84 * (y_g(1) - y_s(1)) + (45 * y_s(2) + 39 * y_g(2)) * T + (10 * y_s(3) - 7 * y_g(3)) * T^2 ...
         + (y_s(4) + 1/2 * y_g(4)) * T^3;
    a6 = 70 * (y_g(1) - y_s(1)) - (36 * y_s(2) + 34 * y_g(2)) * T - (15/2 * y_s(3) - 13/2 * y_g(3)) * T^2 ...
         - (2/3 * y_s(4) + 1/2 * y_g(4)) * T^3;
    a7 = -20* (y_g(1) - y_s(1)) + 10 * (y_s(2) + y_g(2)) * T + 2 * (y_s(3) - y_g(3)) * T^2 +  1/6 * (y_s(4) + y_g(4)) * T^3;
    
    y_i_des = @(t) [a0 + a1 * (t/T) + a2 * (t/T)^2 + a3 * (t/T)^3 + a4 * (t/T)^4 + a5 * (t/T)^5 + a6 * (t/T)^6 + a7 * (t/T)^7;
                               (a1*T^6 + 2*a2*T^5*t + 3*a3*T^4*t^2 + 4*a4*T^3*t^3 + 5*a5*T^2*t^4 + 6*a6*T*t^5 + 7*a7*t^6)/T^7;
                                        (2*a2*T^5 + 6*a3*T^4*t + 12*a4*T^3*t^2 + 20*a5*T^2*t^3 + 30*a6*T*t^4 + 42*a7*t^5)/T^7;
                                                     (6*a3*T^4 + 24*a4*T^3*t + 60*a5*T^2*t^2 + 120*a6*T*t^3 + 210*a7*t^4)/T^7;
                                                                   (24*a4*T^3 + 120*a5*T^2*t + 360*a6*T*t^2 + 840*a7*t^3)/T^7];

end

