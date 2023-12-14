function [r] = NewRadius(Th,Nodes,qo_i,r_min,decay)
    % Find the problem Dimensions
    Dims = 3;

    % Find the Magnitude of the Obstacle velocity
    velocity = sqrt(sum(qo_i(Dims+1:end).^2)); % this must change w/ 3D
    
    % Create an Additional Radius Vector
    Tvec = linspace(0,Th,Nodes);
    
    % Change in radius based on velocity
    delta = velocity*Tvec(2);
    
    % Preallocate the additional radius vector 
    adt_radius = zeros(1,21);
    
    % Update the additional radius vector
    for i = 2:Nodes
        adt_radius(i) = adt_radius(i-1) + delta*decay^i;
    end
    
    % Update the Radius Vector Based on the look ahead
    r = r_min + adt_radius;
end


