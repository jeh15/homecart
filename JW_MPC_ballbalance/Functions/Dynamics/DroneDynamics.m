function dq = DroneDynamics(t,q,t_mpc,u_mpc,A,B,th)
    % Interpolate the control inputs
    u = interp1(t_mpc,u_mpc.',t).'; 
    
    % disp('size(A)')
    % size(A)
    % 
    % disp('size(q)')
    % size(q)
    % 
    % disp('size(B)')
    % size(B)
    % 
    % disp('size(u)')
    % size(u)
    
    
    % Setup for ODE45
    dq = A*q + B*u;
end