function [xout,u_new,exitflag] = smpc2_4d(x1,x2,x3,x4,xt)            

%==========================================================================    
%   Traj Opt settings
%==========================================================================    

    N      = 10;             % Number of nodes (horizon)
    Th     = 0.1;            % MPC Time horizon (prev 13)

    xmeasure = [x1 x2,x3,x4];      % state
    u0   = 0.6*ones(1,N);    % initial input guess - [u(N), beta]
    beta0   = 0.6;           % initial input guess - [u(N), beta]

%==========================================================================    
%   Cost function
%==========================================================================    

    costQ = 0.0*[1  0  0  0; 
                 0 .1  0  0;
                 0  0 .1  0;
                 0  0  0 .1];
    costR = 1e-8;        % last 1e-8

%==========================================================================    
%   SMPC settings
%==========================================================================    

    sig = 0.0000000001;
    % sig = 0.05*50;  %10x   % covariance matrix with sigma^2 (here: uncertainty considered)
    x1_limit = [-0.1,0.1]; % initially 0.08 - now 0.1
    state = 1;        % 1,2,3,4 - position,velocity,acceleration,jerk

%==========================================================================    
% optimization options
%==========================================================================    
    tol_opt       = 1e-8; 
    options = optimset('Display','off',...
        'TolFun', tol_opt,...
        'MaxIter', 10000000,...
        'Algorithm', 'interior-point',...
        'FinDiffType', 'forward',...
        'RelLineSrchBnd', [],...
        'RelLineSrchBndDuration', 1,...
        'TolConSQP', 1e-6,...
        'MaxFunEvals',2*1e3);

    % initial optim options------------------------------------------------
    % options = optimset('Display','off',...
    %     'TolFun', tol_opt,...
    %     'MaxIter', 10000000,...
    %     'Algorithm', 'interior-point',...
    %     'FinDiffType', 'forward',...
    %     'RelLineSrchBnd', [],...
    %     'RelLineSrchBndDuration', 1,...
    %     'TolConSQP', 1e-6,...
    %     'MaxFunEvals',1e4);    

%==========================================================================
%                   System matrices
%==========================================================================    

    golf = 0;
    if golf ==1
        m   = 0.046;       % Mass [kg]
        rr   = 0.021;       % Radius [m]
        Jz  = (2/5)*m*rr^2;    % Moment of inertia [kg.m2]
    else
        m   = 0.003;       % Mass [kg]
        rr   = 0.02;       % Radius [m]
        Jz  = (2/3)*m*rr^2;    % Moment of inertia [kg.m2]
    end
    g   = 9.81;        % Gravity [m/s2]
    K = m*g / (m+(Jz/rr^2));
    
    Ac = [0 1 0 0;
          0 0 K 0;
          0 0 0 1;
          0 0 0 -15]; % originally -31 newish -17 BETTER = -15
    Bc = [ 0 ;
           0 ;
           0 ;
           10 ]; % originally 29 newish 16 BETTER = 10

    Cc = [1 0 0 0];
    Dc = 0;
    [sysd,G] = c2d(ss(Ac,Bc,Cc,Dc),Th,'zoh');
    sysA = sysd.A;
    sysB = sysd.B;

%==========================================================================

    params.x1_limit = x1_limit;
    params.xt = xt;
    params.sysA = sysA;
    params.sysB = sysB;
    params.Th = Th;
    params.state = state;
    params.costQ = costQ;
    params.costR = costR;

%==========================================================================
    
    warning off all

%==========================================================================        

    [u_new, V_current, exitflag, output] = solveOptimalControlProblem ...
        (@runningcosts, @constraints, ...
        @system, @cov_propagation, ...
        N, xmeasure, [u0,beta0], Th, ...
        sig, params, ...
        options);


    xout = computeOpenloopSolution(@system, N, Th, xmeasure, u_new(1:end-1), ...
                                         sig, params);

end

%==========================================================================
%                           solve optimal control problem
%==========================================================================

function [u, V, exitflag, output] = solveOptimalControlProblem ...
    (runningcosts, constraints, ...
    system, cov_propagation, N, x0, ubeta, Th, sig, params, ...
    options)
    
    u0 = ubeta(1:end-1);
    beta0 = ubeta(end);
    
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, Th, x0, u0, ...
                                 sig, params);

    % Set control and linear bounds
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [];
    ub = [];

    
    % Solve optimization problem
    % tic
    [u, V, exitflag, output] = fmincon(@(u) costfunction(runningcosts, system, N, Th, x0, u, sig, params), ...
        [u0,beta0], ...
        A, b, Aeq, beq, lb, ub, ...
        @(u) nonlinearconstraints(constraints, system, cov_propagation, N, Th, x0, u, sig, params), options);
    % toc

    % disp(exitflag)
    % u = u(1:end-1);
end

%==========================================================================
%                           cost function
%==========================================================================

function cost = costfunction(runningcosts, system, ...
                    N, Th, x0, u, ...
                    sig, params)
    cost = 0;


    beta = u(end);
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, Th, x0, u, ...
                                sig, params);
    for k=1:N
        cost = cost+runningcosts(x(k,:), u(:,k),params);
    end
    cost = cost-beta;
end

%==========================================================================
%                           non linear constraints
%==========================================================================

function [c,ceq] = nonlinearconstraints(constraints, ...
    system, cov_propagation, ...
    N, Th, x0, u, sig, params)

    beta = u(end);

    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, Th, x0, u, ...
                                sig, params);
    c = [];
    ceq = [];
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%% constraint tightening computation (& constraint generation)%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%==========================================================================
%       SMPC settings
%==========================================================================            
    
    % compute covariance matrix propagation
    sigma_e = cov_propagation(N,sig,params);    
    g1 = [0;0;0;0];              % constraint: g*x < h -> x1 < x1_limit    
    g1(params.state) = -1; % min

    g2 = [0;0;0;0];              % constraint: g*x < h -> x1 < x1_limit    
    g2(params.state) = 1; % min

    K = [0,0,0,0];
    
    umax = 10.0;

    betamax = 1.0;
    betamin = 0.5;

    for k=1:N               
        gamma1 = sqrt(2*g1'*sigma_e(:,:,k)*g1)*erfinv(2*beta-1);                   % constraint tightening
        gamma2 = sqrt(2*g2'*sigma_e(:,:,k)*g2)*erfinv(2*beta-1);                   % constraint tightening
        [cnew, ceqnew] = constraints(x(k,:),u(:,k), gamma1, gamma2, K, params);   % generate constraints

        c(end+1) = u(k) - umax;
        c(end+1) = -u(k) - umax;


        c = [c cnew];
        ceq = [ceq ceqnew];
    end
    c(end+1) =  u(end) - betamax;
    c(end+1) = -u(end) + betamin;


    c = [c cnew];
    ceq = [ceq ceqnew];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%% end of constraint tightening %%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

%==========================================================================
%                           compute open loop solution
%==========================================================================

function x = computeOpenloopSolution(system, N, Th, x0, u, ...
                                      sig, params)
    x(1,:) = x0;

    uncertainty_flag = 0;
    for k=1:N
        x(k+1,:) = dynamic(system, Th, x(k,:), u(:,k), ...
                              uncertainty_flag, sig, params);
    end
end

%==========================================================================
%                           dynamic
%==========================================================================

function [x] = dynamic(system, Th, ...
             x0, u, apply_flag, sig, params)
    x = system(x0, u, Th, apply_flag, sig, params);
end

%==========================================================================
%                           running costs
%==========================================================================

function cost = runningcosts(x, u,params)
    
    % xt = params.xt;

    Q = params.costQ;
    R = params.costR;
    xt = params.xt;
    xd = [xt,0,0,0];
    cost = (x-xd)*Q*(x-xd)' + u(1)*R*u(1)';
    
end


%==========================================================================
%                           constraints
%==========================================================================

function [c,ceq] = constraints(x, u, gamma1, gamma2, K, params)

    x1_limit = params.x1_limit;                       % get x1 constraint    
    c   = [];
   
%==========================================================================
%       SMPC settings
%==========================================================================            
    % Chance Constraint
    % gamma1 - min , gamma2 - max
    c(end+1) = -x(params.state) + x1_limit(1) + gamma1;           % -x+Min+Del<0 --> x>Min+Del
    c(end+1) =  x(params.state) - x1_limit(2) + gamma2;            %  x<Max-Del

    ceq = [];

end

%==========================================================================
%                           system
%==========================================================================

function y = system(x, u, Th, apply_flag, sig, params)
    % apply_flag: 1) noise is applied for real system; 0) no noise for prediction
    A = params.sysA;
    B = params.sysB;
    K = [0,0,0,0];
    
    y = A*x'+B*(u(1,1) - K*[x(1); x(2); x(3); x(4)]);


    if apply_flag == 1
%         D = [1 0; 0 1];
        D = [0 0 0 0;
             0 1 0 0;
             0 0 0 0;
             0 0 0 0];
        w = [0;0;0;0];
        % Gaussian noise with variance sig^2
        % w(1) = normrnd(0,sig);
        w(2) = normrnd(0,sig);
        % determine next state with uncertainty
        y = y + D*w;
    end

    y = y';    
end

%==========================================================================
%                           cov propagation
%==========================================================================

function sigma_e = cov_propagation(N, sig, params)
    w_cov = [sig^2     0     0     0;
                 0 sig^2     0     0;
                 0     0 sig^2     0;
                 0     0     0 sig^2];

    A = params.sysA;
    B = params.sysB;
%==========================================================================
%       SMPC settings
%==========================================================================            
    
    K = [0 0 0 0];    
    D = [0 0 0 0;
         0 1 0 0;
         0 0 0 0;
         0 0 0 0];

    phi = A-B*K;

    sigma_e = zeros(4,4,N);

    for i = 2:N
        sigma_e(:,:,i) = phi*sigma_e(:,:,i-1)*phi' + D*w_cov*D';
    end
    
end