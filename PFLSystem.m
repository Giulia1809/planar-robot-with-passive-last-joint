classdef PFLSystem < matlab.mixin.Copyable
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        K;
        psi;
        g0;
        q; %vector [x, y, theta]
        q_d; %vector [x_d, y_d, theta_d]
        t;
    end
    
    methods
        function obj = PFLSystem(psi, l_m, q, q_d)
            %UNTITLED2 Construct an instance of this class
            %   Detailed explanation goes here
            obj.K = 2 * l_m / 3;
            obj.psi = psi;
            obj.g0 = 9.81 * cos(psi); 
            obj.q_d = q_d;
            obj.q = q;
            obj.t = 0;
        end
        
        function obj = Integrate(obj, a_in, step)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            %Euler integration :)
%             a_theta = (1 / obj.K) * (sin(obj.q(3)) * a_in(1) - cos(obj.q(3)) * (a_in(2) + obj.g0));
%             a = [a_in; a_theta];
%             obj.q_d = obj.q_d + step * a;
%             obj.q = obj.q + step * obj.q_d;
%             obj.q(3) = wrapToPi(obj.q(3));
%             obj.t = obj.t + step;
            
            %Runge-kutta
            
            function x_dot = dynamic(t, x)
                
                obj.q = [x(1:2); wrapToPi(x(3))];
                obj.q_d = x(4:6);
                a_theta = (1 / obj.K) * (sin(obj.q(3)) * a_in(1) - cos(obj.q(3)) * (a_in(2) + obj.g0));
                a = [a_in; a_theta];

                x_dot = [obj.q_d; a];
            end
            
            [t, x] = ode45(@dynamic, [obj.t obj.t + step], [obj.q; obj.q_d]);
            obj.t = t(end);
            x = x(end,:)';
            obj.q = [x(1:2); wrapToPi(x(3))];
            obj.q_d = x(4:6);
        end
    end
end

