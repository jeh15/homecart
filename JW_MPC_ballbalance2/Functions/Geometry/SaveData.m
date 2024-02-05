function [Time,Drone,Obstacle] = SaveData(Time,Drone,Obstacle)
   
    % Update the Initial Conditions
    % Drone point is based on the NonLinear Dynamics
    Drone.q_i = Drone.q_sim(end,:).';
    Obstacle.q_i = Obstacle.q_sim(end,:).';
    
    % Update The Measured Drone Properties
    Drone.q_total = [Drone.q_total(:,1:end-1), Drone.q_sim.'];
    Drone.u_total = [Drone.u_total(:,1:end-1), Drone.u_sim.'];

    % Update The Measured Obsticle Properties
    Obstacle.q_total = [Obstacle.q_total(:,1:end-1), Obstacle.q_sim.'];
    
    % Update Time Parameters
    Time.total = [Time.total(1:end-1), Drone.t_sim.'];
    Time.current = Drone.t_sim(end);
end

