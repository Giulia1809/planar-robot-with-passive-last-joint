classdef Robot < matlab.mixin.Copyable
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        L1;
        L2;
        L3;
        psi;
        I1;
        I2;
        I3;
        l1;
        l2;
        l3;
        m1;
        m2;
        m3;
    end
    
    methods
        function obj = Robot(L1, L2, L3, m1, m2, m3, psi)
            %UNTITLED5 Construct an instance of this class
            %   Detailed explanation goes here
            obj.L1 = L1;
            obj.L2 = L2;
            obj.L3 = L3;
            obj.l1 = L1/2;
            obj.l2 = L2/2;
            obj.l3 = L3/2;
            obj.m1 = m1;
            obj.m2 = m2;
            obj.m3 = m3;
            obj.I1 = m1*L1^2/3;
            obj.I2 = m2*L2^2/3;
            obj.I3 = m3*L3^2/3;
            obj.psi = psi;
        end
        
        function p = DirectKinematics(obj, q)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            p = [ obj.L1 * cos(q(1)) + obj.L2 * cos(q(1) + q(2));
                  obj.L1 * sin(q(1)) + obj.L2 * sin(q(1) + q(2))];
        end
        
        function q = InverseKinematics(obj, p)
            c2 = (p(1)^2 + p(2)^2 -obj.L1^2 -obj.L2^2)/(2 * obj.L1 * obj.L2);
            s2 = sqrt(1-c2^2);
            q2 = atan2(s2, c2);
            q1 = wrapToPi( atan2(p(2), p(1)) - atan2(obj.L2 * s2, obj.L1 + obj.L2 * c2));
            q = [q1; q2];
        end
        
        function tau = ForceTransformation(obj, p, theta_3, p_d, theta_3_d, a)
            q = obj.InverseKinematics(p);
            q = [q; wrapToPi(theta_3 - q(1) - q(2))];
           
            J = [-obj.L2*sin(q(1) + q(2)) - obj.L1*sin(q(1)), -obj.L2*sin(q(1) + q(2));
                  obj.L2*cos(q(1) + q(2)) + obj.L1*cos(q(1)),  obj.L2*cos(q(1) + q(2))];
            q_d = pinv(J)*p_d;
            q_d = [q_d; theta_3_d - q_d(1) - q_d(2)];
            
            a_theta_3 = 1/(2 * obj.L3 / 3) * (sin(theta_3)*a(1) - cos(theta_3)*(a(2)+ (9.81*cos(obj.psi))));
            J_d = [ - obj.L2*cos(q(1) + q(2))*(q_d(1) + q_d(2)) - obj.L1*cos(q(1))*q_d(1), -obj.L2*cos(q(1) + q(2))*(q_d(1) + q_d(2));
                    - obj.L2*sin(q(1) + q(2))*(q_d(1) + q_d(2)) - obj.L1*sin(q(1))*q_d(1), -obj.L2*sin(q(1) + q(2))*(q_d(1) + q_d(2))];
            q_d_d = pinv(J) * (a - J_d * q_d(1:2));
            q_d_d = [q_d_d; a_theta_3 - q_d_d(1) - q_d_d(2)];
            
            B = obj.GetB(q);
            c = obj.GetC(q, q_d);
            g = obj.GetG(q);
            tau =  B * q_d_d + c + g;
        end
        
        function B = GetB(obj, q)
            B = [obj.I1 + obj.I2 + obj.I3 + obj.L1^2*obj.m2 + obj.L1^2*obj.m3 + obj.L2^2*obj.m3 + obj.l1^2*obj.m1 + obj.l2^2*obj.m2  + obj.l3^2*obj.m3 + 2*obj.L1*obj.L2*obj.m3*cos(q(2)) + 2*obj.L1*obj.l2*obj.m2*cos(q(2)) + 2*obj.L2*obj.l3*obj.m3*cos(q(3)) + 2*obj.L1*obj.l3*obj.m3*cos(q(2) + q(3)),       obj.I2 + obj.I3 + obj.L2^2*obj.m3 + obj.l3^2*obj.m3 + obj.L1*obj.L2*obj.m3*cos(q(2)) + obj.l2^2*obj.m2 + obj.L1*obj.l2*obj.m2*cos(q(2))  + 2*obj.L2*obj.l3*obj.m3*cos(q(3)) + obj.L1*obj.l3*obj.m3*cos(q(2) + q(3)),         obj.I3 + obj.l3^2*obj.m3 + obj.L2*obj.l3*obj.m3*cos(q(3)) + obj.L1*obj.l3*obj.m3*cos(q(2) + q(3));
                                                                                       obj.I2 + obj.I3 + obj.L2^2*obj.m3 + obj.l2^2*obj.m2 + obj.l3^2*obj.m3 + obj.L1*obj.L2*obj.m3*cos(q(2)) + obj.L1*obj.l2*obj.m2*cos(q(2)) + 2*obj.L2*obj.l3*obj.m3*cos(q(3)) + obj.L1*obj.l3*obj.m3*cos(q(2) + q(3)),                                                                                                                  obj.I2 + obj.I3 + obj.L2^2*obj.m3 + obj.l2^2*obj.m2 + obj.l3^2*obj.m3 + 2*obj.L2*obj.l3*obj.m3*cos(q(3)),                                                 obj.I3 + obj.l3^2*obj.m3 + obj.L2*obj.l3*obj.m3*cos(q(3));
                                                                                                                                                                                                        obj.I3 + obj.l3^2*obj.m3 + obj.L2*obj.l3*obj.m3*cos(q(3)) + obj.L1*obj.l3*obj.m3*cos(q(2) + q(3)),                                                                                                                                                                 obj.I3 + obj.l3^2*obj.m3 + obj.L2*obj.l3*obj.m3*cos(q(3)),                                                                                  obj.I3 + obj.l3^2*obj.m3];
        end
        
        function c = GetC(obj, q, q_d)
            c = [-obj.L1*obj.l3*obj.m3*q_d(2)^2*sin(q(2) + q(3)) - obj.L1*obj.l3*obj.m3*q_d(3)^2*sin(q(2) + q(3)) - obj.L1*obj.L2*obj.m3*q_d(2)^2*sin(q(2)) - obj.L1*obj.l2*obj.m2*q_d(2)^2*sin(q(2)) - obj.L2*obj.l3*obj.m3*q_d(3)^2*sin(q(3)) - 2*obj.L1*obj.l3*obj.m3*q_d(1)*q_d(2)*sin(q(2) + q(3)) - 2*obj.L1*obj.l3*obj.m3*q_d(1)*q_d(3)*sin(q(2) + q(3)) - 2*obj.L1*obj.l3*obj.m3*q_d(2)*q_d(3)*sin(q(2) + q(3)) - 2*obj.L1*obj.L2*obj.m3*q_d(1)*q_d(2)*sin(q(2)) - 2*obj.L1*obj.l2*obj.m2*q_d(1)*q_d(2)*sin(q(2)) - 2*obj.L2*obj.l3*obj.m3*q_d(1)*q_d(3)*sin(q(3)) - 2*obj.L2*obj.l3*obj.m3*q_d(2)*q_d(3)*sin(q(3));
                                                                                                                                                                                                                                                                                                                                             obj.L1*obj.l3*obj.m3*q_d(1)^2*sin(q(2) + q(3)) + obj.L1*obj.L2*obj.m3*q_d(1)^2*sin(q(2)) + obj.L1*obj.l2*obj.m2*q_d(1)^2*sin(q(2)) - obj.L2*obj.l3*obj.m3*q_d(3)^2*sin(q(3)) - 2*obj.L2*obj.l3*obj.m3*q_d(1)*q_d(3)*sin(q(3)) - 2*obj.L2*obj.l3*obj.m3*q_d(2)*q_d(3)*sin(q(3));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                obj.l3*obj.m3*(obj.L1*q_d(1)^2*sin(q(2) + q(3)) + obj.L2*q_d(1)^2*sin(q(3)) + obj.L2*q_d(2)^2*sin(q(3)) + 2*obj.L2*q_d(1)*q_d(2)*sin(q(3)))];
        end
        
        function g = GetG(obj, q)
            g = [                                                                                                                           -(981*cos(obj.psi)*(obj.l1*obj.m1*cos(q(1)) + obj.l3*obj.m3*cos(q(1) + q(2) + q(3)) + obj.L2*obj.m3*cos(q(1) + q(2)) + obj.l2*obj.m2*cos(q(1) + q(2)) + obj.L1*obj.m2*cos(q(1)) + obj.L1*obj.m3*cos(q(1))))/100;
                 -(981*obj.L2*obj.m3*cos(obj.psi + q(1) + q(2)))/200 - (981*obj.l3*obj.m3*cos(obj.psi + q(1) + q(2) + q(3)))/200 - (981*obj.l2*obj.m2*cos(obj.psi + q(1) + q(2)))/200 - (981*obj.L2*obj.m3*cos(q(1) - obj.psi + q(2)))/200 - (981*obj.l3*obj.m3*cos(q(1) - obj.psi + q(2) + q(3)))/200 - (981*obj.l2*obj.m2*cos(q(1) - obj.psi + q(2)))/200;
                                                                                                                                                                                                                                                           -(981*obj.l3*obj.m3*(cos(obj.psi + q(1) + q(2) + q(3)) + cos(q(1) - obj.psi + q(2) + q(3))))/200];
        end
        
    end
end

