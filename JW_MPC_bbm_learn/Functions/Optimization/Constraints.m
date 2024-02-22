function [x,u,dv] = Constraints(Th,Nodes,Ad,Bd,xd_lb,xd_ub,vd_lb,vd_ub,du_max)
    %% Develop the Nonlinear Constraints
    % Dermine the Lengths of the Input Matricies & Nodes
     A_len = size(Ad,2);
     B_len = size(Bd,2);
     Dims = 1;

     % Time Variable
     Ts = sym('Ts',[1 1],'real');                   % Calculation Time
     
     % System States
     x_ic = sym('xic',[A_len 1],'real');            % Initial Condition Vector
     x = sym('x',[A_len Nodes],'real');             % Optimization State Matrix
     % x_prev = sym('x_prev',[A_len Nodes],'real');   % Previous Optimization State Matrix
     u = sym('u',[B_len Nodes],'real');             % Control Input Matrix  
     k_model = sym('k_model',[1,1],'real');       % ddx-ball = k-model * th-board

     
     % Design Vector
     dv = [x(:);u(:)]; % Design Vector has dt in it   
     
     %% Equality Constraint
     disp('Developing Equality Constraints...')
     % Impliment Dynamics x_dot = Ax + Bu
    
     Ad = [0 1 0 0;
           0 0 k_model(1) 0;
           0 0 0 1;
           0 0 0 -15]; % originally -31 newish -17 BETTER = -15

     % Ad(2,3) = k_model;

     dx = Ad*x + Bd*u;
    
    

     dt = Th/(Nodes-1);
     
     % Integration Method
     % Explicit Euler
     ceq_defect = x(:,2:end) - x(:,1:end-1) - dx(:,1:end-1)*dt;
     
     % Task Constraints - Initial and Final States, Obstacle Avoidance
     ceq_ic = x(:,1) - x_ic;
     
     % Combine the Equality Constraints
     ceq = [ceq_defect(:); ceq_ic(:)];

     % Seperate out the A and b
     Aeq = jacobian(ceq,dv);
     beq = -subs(ceq,dv,zeros(size(dv)));
     
     %% Inequality Constraints
     disp('Developing Inequality Constraints...')
     % Cartesian Bound equalities
     ciq_x_lb = -x(1:Dims,:) + xd_lb;
     ciq_x_ub = x(1:Dims,:) - xd_ub;
     
     % Velocity Bound equalities
     ciq_v_lb = -x(Dims+1:2,:) + vd_lb;
     ciq_v_ub = x(Dims+1:2,:) - vd_ub;

     % control input delta-node max
     ciq_u_lb = -(u(2:end) - u(1:end-1)) - du_max;
     ciq_u_ub = (u(2:end) - u(1:end-1)) - du_max;
     
     
     
     % Linearized Ball Constraint - Based on initial state
     % ciq_ball = AvoidanceConstraint(x_ic, x, x_prev, xo_ic, r1, Th, Ts, Nodes);
     
     % Slack Variable Must be positive
     % ciq_slack = -s;     
     
     % Constraint the Slack Variable value to be > 0 when w/in the outter radius
     % ciq_cost = p.*(ciq_ball + (r2 - r1)) - s;     

     % Combine the Inequality Constraints
     ciq = [ciq_x_lb(:); ciq_x_ub(:); ciq_v_lb(:); ciq_v_ub(:); ciq_u_lb(:); ciq_u_ub(:)];
     
     % Seperate out the A and b
     Aiq = jacobian(ciq,dv);
     biq = -subs(ciq,dv,zeros(size(dv)));     
    
     %% Create functions    
     disp('Generating Equality Constraint Function...')     
     matlabFunction(Aeq,beq,'File','/home/orl/repository/homecart/JW_MPC_bbm_learn/Functions/AutoGenerated/EqualityConstraints','vars',{x_ic,k_model},'Outputs',{'Aeq','beq'}); 
     disp('Generating Inequality Constraint Function...') 
     matlabFunction(Aiq,biq,'File','/home/orl/repository/homecart/JW_MPC_bbm_learn/Functions/AutoGenerated/InequalityConstraints','vars',{x_ic},'Outputs',{'Aiq','biq'}); 
end


