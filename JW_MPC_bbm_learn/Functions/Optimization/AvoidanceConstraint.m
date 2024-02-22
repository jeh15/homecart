function [ciq_ball] = AvoidanceConstraint(drone_states_ic, drone_states_future, drone_states_future_old, obstacle_states_ic, r, Th, Ts, Nodes)
    % Uses the current position of the drone, and the expected position of
    % the obstace to develop linear constraints keeping the drone out of
    % the circle by linearizing minimin radius point along the distance vector    

    % Create Time Vectors
    dt = Th/(Nodes-1);
    Tvec = linspace(0,Th,Nodes);
    Tvec_new = linspace(Ts,Th+Ts,Nodes);
    
    % Grab the Position Data
    vec_size = size(drone_states_ic,1)/2;
    q_ic = drone_states_ic(1:vec_size);
    qo_ic = obstacle_states_ic(1:vec_size);
    q = drone_states_future(1:vec_size,:);
    q_prev = drone_states_future_old(1:vec_size,:);
    
    % Update the previous trajectory based on measured time & states
%     q_diff = (q_prev(:,2:end) - q_prev(:,1:end-1))*Ts/dt;
%     q_prev_interp = [q_prev(:,1:end-1) + q_diff, q_prev(:,end)];
%     q_fc = q_prev_interp - q_prev_interp(:,1) + q_ic;
    q_fc = q_prev - q_prev(:,1) + q_ic;
    
    % Measured Obstacle Velocity
    dqo_ic = obstacle_states_ic(vec_size+1:end);
    
    % Use Initial Velocity to Estimate the Obstacle Positions
    qo_fc = qo_ic + Tvec.*dqo_ic;
    
    % Distance Between Objects
    q_dist = q_fc - qo_fc;
    
    % Find the Radial Intercept Points
    for i = 1:Nodes
        u(:,i) = q_dist(:,i)/norm(q_dist(:,i));
    end
    intercept = qo_fc + r.*u;
    
    % Find the Vector between the Agent and Obstacle
    vector = (q_fc - qo_fc);
    
    % Constraint Plane Equation  
    ciq_ball = -dot(vector,q - intercept);
end

