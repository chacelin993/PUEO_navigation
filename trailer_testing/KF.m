classdef KF < handle
    properties
        x;
        P;
        g;
        sa2; % sigma_a squared
        sg2; % sigma_g sqaured
    end
    properties (Dependent)
        q;
        p;
        v;
    end
    methods
        function propagate(obj,z,dt)
            w=z(1:3);
            a=z(4:6);
            w_norm=norm(w);
            quat = quaternion(obj.q.');
            rot = quat2rotm(quat);
            obj.x(5:7) = obj.p + obj.v*dt - ...
                        obj.g*dt^2/2 ...;
                        + rot*a*dt^2/2;
            obj.x(8:10) = obj.v - obj.g*dt + ...
                rot*a*dt;
            if w_norm~=0
                integrator0=quaternion(cos(w_norm/2*dt), ...
                    w(1)/w_norm*sin(w_norm/2*dt), ...
                    w(2)/w_norm*sin(w_norm/2*dt), ...
                    w(3)/w_norm*sin(w_norm/2*dt));
            else
                integrator0=quaternion(1,0,0,0);
            end
            quat = quatmultiply(quat, integrator0);
%             quat = quatmultiply(integrator0, quat);
            rot1 = quat2rotm(quat);
            obj.x(1:4) = compact(quat);
            dR = rot1*transpose(rot);
            Phi = eye(9);
            Phi(1:3,1:3) = dR.';
            Phi(4:6,1:3) = -1/2*rot*skew(a*dt^2);
            Phi(7:9,1:3) = -rot*skew(a*dt);
            Phi(4:6,7:9) = eye(3)*dt;
            G = zeros(9,6);
            G(1:3,1:3) = -transpose(dR)*Jr(-w*dt)*dt;
            G(4:6,4:6) = -1/2*rot*dt^2;
            G(7:9,4:6) = -rot*dt;
            Q = eye(6,6);
            Q(1:3,1:3) = Q(1:3,1:3)*obj.sg2;
            Q(4:6,4:6) = Q(4:6,4:6)*obj.sa2;
            obj.P = Phi*obj.P*transpose(Phi) + G*Q*transpose(G);
        end
        function update(obj, z, R) % this z is the measurement of GPS
            z = z.';
            H = zeros(3,9);
            H(:,4:6) = eye(3);
            K = obj.P*H.'/(H*obj.P*H.' + R);
            dx = K*(z-obj.p);
            dq = [1;dx(1:3)/2];
            dq = dq/norm(dq);
            obj.x(1:4) = compact(quatmultiply(quaternion(dq.'),quaternion(obj.x(1:4).')));
            % obj.x(1:4) = compact(quatmultiply(quaternion(obj.x(1:4).',quaternion(dq.'))));
            obj.x(5:10) = obj.x(5:10) + dx(4:9);
            obj.P = obj.P - K*H*obj.P;
        end
        function val = get.q(obj)
            val = obj.x(1:4);
        end
        function val = get.p(obj)
            val = obj.x(5:7);
        end
        function val = get.v(obj)
            val = obj.x(8:10);
        end
    end
end
function val = Jr(theta)
theta_norm=norm(theta);
% n = theta/theta_norm; % rotation axis
% val = sin(theta_norm)/theta_norm * eye(3) + ...
%     (1 - sin(theta_norm)/theta_norm)*n*transpose(n) ...
%     - (1-cos(theta_norm))/theta_norm * skew(n);
val = eye(3) - (1 - cos(theta_norm))/theta_norm^2 * skew(theta) + ...
    (theta_norm - sin(theta_norm))/theta_norm^3 * skew(theta)*skew(theta);
end
function val = skew(v)
val = zeros(3,3);
val(1,2)=-v(3);
val(2,1)=v(3);
val(1,3)=v(2);
val(3,1)=-v(2);
val(2,3)=-v(1);
val(3,2)=v(1);
end