function [Tsim,qd_sim] = Simulate(Th,Ts,Nodes,qd_i,ud,Ad,Bd,th)
    % Options
    options = odeset('RelTol',1e-5);

    % Time vector
    Tvec = linspace(0,Th,Nodes);
    % Update Initial Conditions at Update Rate time
    [Tsim, qd_sim] = ode45(@(t,q) DroneDynamics(t,q,Tvec,ud,Ad,Bd,th), [0 Ts], qd_i, options);
    
    % Develop the Obstacle Motion
end

