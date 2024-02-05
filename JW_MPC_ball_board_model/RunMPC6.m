function [qd, ud] = RunMPC6(Th,Nodes,qd_i,qd_des,xd_lb,xd_ub)
   % Optimization Options
    options = optimset('Display','off');

    % Neither Radius Increases
    % r1 = r_min.*ones(1,Nodes);
    % r2 = r_min2.*ones(1,Nodes); 
    
    % Develop the Constraints Matrix and Vector
    [Aeq,beq] = EqualityConstraints(qd_i);
    [Aiq,biq] = InequalityConstraints(qd_i);
    
    % Develop The Objective Function Matrix and Vector
    [H,f] = ObjectiveFunction(qd_des);
    
    % Find the decision vector   
    design_vector = quadprog(H,f,Aiq,biq,Aeq,beq,[],[],[],options);

    % Initialize trajectory matrix
    dt = Th/(Nodes - 1);
    trajectory = zeros(Nodes,33);    
    
    % Seperate the states in the design vector
    M = size(qd_i,1);           
    O = 1;         % Size of bounding vector
    qd = zeros(M,Nodes);        % Initialize the desired states
    ud = zeros(O,Nodes);        % Initialize the desired control inputs
    for i = 1:M
        try
            qd(i,:) = design_vector(i:M:M*Nodes).';
        catch     
            if qd_i(1) <= xd_lb(1) || qd_i(1) >= xd_ub(1) 
                warning('Cartesian Bounds Violated, Optimization did not produce results');
            else
                warning('Optimization can not find a real solution based on the system dynamics');
            end
            % qd(1:2,:) = qd_i(1:2).*ones(M,Nodes);
        end
    end
    
    % Seperate the forces in the design vector
    for j = 1:O
        try
            ud(j,:) = design_vector(M*Nodes+j:O:O*Nodes+M*Nodes).';
        catch
        end
    end
    

    
    % Replace variables with the known results [dt, x, dx, y, dy, z, dz] = [1, 2, 3, 10, 11, 18, 19]
    % trajectory(:,1) = dt;
    % trajectory(:,2:3) = [qd(1,:); qd(3,:)].';
    % trajectory(:,10:11) = [qd(2,:); qd(4,:)].';
    % 
    % % Find further numeric derivatives
    % for i = 1:6
    %     trajectory(:,i+3) = gradient(trajectory(:,i+2));
    %     trajectory(:,i+11) = gradient(trajectory(:,i+10));
    % end
    % trajectory = trajectory(1:end,:);
    % % Create a csv file for python
    % writematrix(trajectory,'TEST.csv')
end
