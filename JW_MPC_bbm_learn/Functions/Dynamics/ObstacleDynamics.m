function dq = ObstacleDynamics(t,q,u,A,B)
    % Setup for ODE45
    dq = A*q + B*u;
end