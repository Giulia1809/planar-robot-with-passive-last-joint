classdef DynamicCompensator < matlab.mixin.Copyable
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        pfl_robot;
        eta;
        xi;
        y;
        y_d;
        y_d_d;
        y_3_d;
        a;
    end
    
    methods
        function obj = DynamicCompensator(pfl_sys, eta, xi)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.pfl_robot = pfl_sys;
            obj.eta = eta;
            obj.xi = xi;
            %equation 13
            obj.y = obj.pfl_robot.q(1:2) + obj.pfl_robot.K * [cos(obj.pfl_robot.q(3)); sin(obj.pfl_robot.q(3))];
            %equation 14
            obj.y_d = obj.pfl_robot.q_d(1:2) + obj.pfl_robot.K * obj.pfl_robot.q_d(3) * [-sin(obj.pfl_robot.q(3)); cos(obj.pfl_robot.q(3))];
            %equation 16
            obj.y_d_d = AngleToRot(obj.pfl_robot.q(3)) * [obj.xi; -obj.pfl_robot.g0 * cos(obj.pfl_robot.q(3))];
            %equation 19
            obj.y_3_d = AngleToRot(obj.pfl_robot.q(3)) * [ obj.eta + obj.pfl_robot.g0 * cos(obj.pfl_robot.q(3)) * obj.pfl_robot.q_d(3); obj.xi * obj.pfl_robot.q_d(3) + obj.pfl_robot.g0 * sin(obj.pfl_robot.q(3)) * obj.pfl_robot.q_d(3)];
            obj.a = [0; 0];
        end
        
        function obj = Integrate(obj, v, step)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            A = obj.GetA();
            R = AngleToRot(obj.pfl_robot.q(3));
            b = obj.GetB();
            
            %equation 21
            sigma = inv(A) * (R' * v - b);
            
            obj.eta = obj.eta + sigma(1) * step;
            obj.xi = obj.xi + obj.eta * step;
            
            %equation 15
            obj.a = R * [obj.xi + obj.pfl_robot.K * obj.pfl_robot.q_d(3)^2; sigma(2)];
            
            obj.pfl_robot.Integrate(obj.a, step);
            
            %equation 13
            obj.y = obj.pfl_robot.q(1:2) + obj.pfl_robot.K * [cos(obj.pfl_robot.q(3)); sin(obj.pfl_robot.q(3))];
            %equation 14
            obj.y_d = obj.pfl_robot.q_d(1:2) + obj.pfl_robot.K * obj.pfl_robot.q_d(3) * [-sin(obj.pfl_robot.q(3)); cos(obj.pfl_robot.q(3))];
            %equation 16
            obj.y_d_d = AngleToRot(obj.pfl_robot.q(3)) * [obj.xi; -obj.pfl_robot.g0 * cos(obj.pfl_robot.q(3))];
            %equation 19
            obj.y_3_d = AngleToRot(obj.pfl_robot.q(3)) * [ obj.eta + obj.pfl_robot.g0 * cos(obj.pfl_robot.q(3)) * obj.pfl_robot.q_d(3); obj.xi * obj.pfl_robot.q_d(3) + obj.pfl_robot.g0 * sin(obj.pfl_robot.q(3)) * obj.pfl_robot.q_d(3)];
        end
        
        function A = GetA(obj)
            A = [1, -(obj.pfl_robot.g0 / obj.pfl_robot.K) * cos(obj.pfl_robot.q(3));
                 0, -(obj.xi + obj.pfl_robot.g0 * sin(obj.pfl_robot.q(3))) / obj.pfl_robot.K];
        end
        
        function b = GetB(obj)
            b = [                                                                     -(2 * obj.pfl_robot.g0 * sin(obj.pfl_robot.q(3)) + obj.xi) * obj.pfl_robot.q_d(3)^2 - ((obj.pfl_robot.g0^2 / obj.pfl_robot.K) * cos(obj.pfl_robot.q(3))^2);
                 2 * (obj.pfl_robot.g0 * cos(obj.pfl_robot.q(3)) * obj.pfl_robot.q_d(3) + obj.eta) * obj.pfl_robot.q_d(3) - (obj.pfl_robot.g0 / obj.pfl_robot.K) * (obj.xi + obj.pfl_robot.g0 * sin(obj.pfl_robot.q(3))) * cos(obj.pfl_robot.q(3))];
        end
    end
end

