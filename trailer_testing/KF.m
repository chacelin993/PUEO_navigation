classdef KF < handle
    properties
        x;
        P;
        g;
        sa2; % sigma_a squared
        sg2; % sigma_g sqaured
        ab; % acc bias
        wb; % gyro bias
        saw2; % sigma ab squared
        sww2; % sigma wb squared
    end
    properties (Dependent)
        q;
        p;
        v;
    end
    methods
        function propagate(obj,z,dt)
            w=z(1:3)-obj.wb;
            a=z(4:6)-obj.ab;
            quat = quaternion(obj.q.');
            rot = quat2rotm(quat);
            obj.x(1:3) = obj.p + obj.v*dt + ...
                        1/2*(rot*a+obj.g)*dt^2;
            obj.x(4:6) = obj.v + (rot*a + obj.g)*dt;
%             w_norm=norm(w);
%             if w_norm~=0
%                 integrator0=quaternion(cos(w_norm/2*dt), ...
%                     w(1)/w_norm*sin(w_norm/2*dt), ...
%                     w(2)/w_norm*sin(w_norm/2*dt), ...
%                     w(3)/w_norm*sin(w_norm/2*dt));
%             else
%                 integrator0=quaternion(1,0,0,0);
%             end
            integrator0 = exp(quaternion([0,w.'*dt/2]));
            quat = quatmultiply(quat, integrator0);
%             quat = quatnormalize(quat);
            obj.x(7:10) = compact(quat);
            dR = quat2rotm(integrator0);
            Phi = eye(15);
            Phi(1:3,4:6) = eye(3)*dt;
            Phi(4:6,7:9) = -rot*skew(a)*dt;
            Phi(4:6,10:12) = -rot*dt;
            Phi(7:9,7:9) = dR.';
            Phi(7:9,13:15) = -eye(3)*dt;
            G = zeros(15,12);
            G(4:15,1:12) = eye(12);
            Q = ones(12,1);
            Q(1:3) = obj.sa2*dt*dt;
            Q(4:6) = obj.sg2*dt*dt;
            Q(7:9) = obj.saw2*dt;
            Q(10:12) = obj.sww2*dt;
            Q = diag(Q);
            obj.P = Phi*obj.P*transpose(Phi) + G*Q*transpose(G);
        end
        function update(obj, z, R) % this z is the measurement of GPS
            z = z.';
            H = zeros(6,15);
            H(1:3,1:3) = eye(3);
            H(4:6,7:9) = eye(3);
            K = obj.P*H.'/(H*obj.P*H.' + R);
            S = zeros(6,1);
            theta_quat = log(quatmultiply(quatconj(quaternion(obj.x(7:10).')),quaternion(z(4:7).')));
            theta = compact(theta_quat);
            S(4:6) = theta(2:end) * 2; % X2 for the theta in quaternion is halved;
            S(1:3) = z(1:3) - obj.x(1:3);
            dx = K * S;
            dq = quaternion([0,dx(7:9).']/2);
            dq = exp(dq);
            obj.x(7:10) = compact(quatmultiply(quaternion(obj.x(7:10).'), dq));
            obj.x(1:6) = obj.x(1:6) + dx(1:6);
            obj.x(11:16) = obj.x(11:16) + dx(10:15);
            obj.P = obj.P - K*H*obj.P;
        end
        function val = get.q(obj)
            val = obj.x(7:10);
        end
        function val = get.p(obj)
            val = obj.x(1:3);
        end
        function val = get.v(obj)
            val = obj.x(4:6);
        end
        function val = get.ab(obj)
            val = obj.x(11:13);
        end
        function val = get.wb(obj)
            val = obj.x(14:16);
        end
    end
end
% function val = Jr(theta)
% theta_norm=norm(theta);
% % n = theta/theta_norm; % rotation axis
% % val = sin(theta_norm)/theta_norm * eye(3) + ...
% %     (1 - sin(theta_norm)/theta_norm)*n*transpose(n) ...
% %     - (1-cos(theta_norm))/theta_norm * skew(n);
% val = eye(3) - (1 - cos(theta_norm))/theta_norm^2 * skew(theta) + ...
%     (theta_norm - sin(theta_norm))/theta_norm^3 * skew(theta)*skew(theta);
% end
function val = skew(v)
val = zeros(3,3);
val(1,2)=-v(3);
val(2,1)=v(3);
val(1,3)=v(2);
val(3,1)=-v(2);
val(2,3)=-v(1);
val(3,2)=v(1);
end