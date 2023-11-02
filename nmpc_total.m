function out = nmpc_total(x1,x2,x3,x4)            
%==========================================================================    
% Goal: Solve the tossing of a mass into a bowl problem
% 
%  - Set system parameters
%  - Set MPC parameters
%  - Run SMPC
%  
%==========================================================================    


%==========================================================================    
% MPC settings
%==========================================================================    
    Tfinal = 1.0*10;         % Tfinal for simulation
    N      = 11;             % Number of nodes (horizon)
    Th     = 0.5;            % MPC Time horizon
    Tc     = 0.1;            % Control Time horizon

    % xmeasure = [xx 0.0 0.0 0.0];     % initial [x dx ddx dddx] (last ur5e link)

    xmeasure = [x1 x2 x3 0];     % initial [x dx ddx dddx] (last ur5e link)

    tmeasure = 0.0;        % initial time (do not change)

    % control input settings
    u0   = 0.01*ones(1,N);  % initial input guess
    ulim = 0.2;

    % task state constraint - set state limits
    sig = 20*.1*.1*0.08;             % sigma of Gaussian distribution -> covariance matrix with sigma^2 (here: uncertainty considered)
    beta = 0.8;            % smpc risk parameter, [0.5 to 0.999] (here: high beta means low risk)
    targ = 2.1; del = 0.1; % ideal 2.1
    x1_limit = [targ-del,targ+del];         % limit for x1 - (chance) constraint -> final velocity
    state = 2;        % 1,2,3,4 - position,velocity,acceleration,jerk

%==========================================================================



%==========================================================================    
% optimization options
%==========================================================================    
    tol_opt       = 1e-8; % 8
    opt_option    = 0;
    iprint        = 10;             % see nmpc.m line 81+
    type          = 'difference equation';
    atol_ode_real = 1e-12;
    rtol_ode_real = 1e-12;
    atol_ode_sim  = 1e-4;
    rtol_ode_sim  = 1e-4;
%==========================================================================        
        

%==========================================================================    
% System Parameters
%==========================================================================    
    
    % q - [x dx ddx dddx] - { x - distance along pan(inclined plane) }
    
    Ac = [0  1  0  0;...
          0  0  1  0;...
          0  0  0  1;...
          0  0  0  0;];
    
    m   = 0.003;            % Mass              [kg]
    R   = 0.02;
    Jz  = .5*m*R^2;    % Moment of inertia [kg.m2]
    g   = 9.81;             % Gravity           [m/s2]
    
    K = m*g / (m+Jz/R^2);
        
    Bc = [0;
          0;
          0;
          K];
    
    Cc = [1 0 0 0];
    Dc = 0;
    
    [sysd,G] = c2d(ss(Ac,Bc,Cc,Dc),Th,'zoh');
    
    sysA = sysd.A;
    sysB = sysd.B;
    
    params.x1_limit = x1_limit;
    params.sysA = sysA;
    params.sysB = sysB;
    params.Th = Th;
    params.ulim = ulim;
%    params.delta = delta;
    params.state = state;
    params.Tc = Tc;
    
    rng('shuffle');                 % random seed
    s = rng;                        % save rng setting
    
%==========================================================================    
% fmincon option assignment
%==========================================================================
    % Determine MATLAB Version and
    % specify and configure optimization method
    vs = version('-release');
    vyear = str2num(vs(1:4));
    if (vyear <= 2007)
        fprintf('MATLAB version R2007 or earlier detected\n');
        fprintf('Starting MPC routine\n');
        if ( opt_option == 0 )
            options = optimset('Display','off',...
                'TolFun', tol_opt,...
                'MaxIter', 2000,...
                'LargeScale', 'off',...
                'RelLineSrchBnd', [],...
                'RelLineSrchBndDuration', 1);
        elseif ( opt_option == 1 )
            error('nmpc:WrongArgument', '%s\n%s', ...
                  'Interior point method not supported in MATLAB R2007', ...
                  'Please use opt_option = 0 or opt_option = 2');
        elseif ( opt_option == 2 )
             options = optimset('Display','off',...
                 'TolFun', tol_opt,...
                 'MaxIter', 2000,...
                 'LargeScale', 'on',...
                 'Hessian', 'off',...
                 'MaxPCGIter', max(1,floor(size(u0,1)*size(u0,2)/2)),...
                 'PrecondBandWidth', 0,...
                 'TolPCG', 1e-1);
        end
    else
    %         fprintf('MATLAB version R2008 or newer detected\n');
        fprintf('Starting MPC routine\n');
        if ( opt_option == 0 )
            options = optimset('Display','off',...
                'TolFun', tol_opt,...
                'MaxIter', 10000,...
                'Algorithm', 'active-set',...
                'FinDiffType', 'forward',...
                'RelLineSrchBnd', [],...
                'RelLineSrchBndDuration', 1,...
                'TolConSQP', 1e-6);
        elseif ( opt_option == 1 )
            options = optimset('Display','off',...
                'TolFun', tol_opt,...
                'MaxIter', 2000,...
                'Algorithm', 'interior-point',...
                'AlwaysHonorConstraints', 'bounds',...
                'FinDiffType', 'forward',...
                'HessFcn', [],...
                'Hessian', 'bfgs',...
                'HessMult', [],...
                'InitBarrierParam', 0.1,...
                'InitTrustRegionRadius', sqrt(size(u0,1)*size(u0,2)),...
                'MaxProjCGIter', 2*size(u0,1)*size(u0,2),...
                'ObjectiveLimit', -1e20,...
                'ScaleProblem', 'obj-and-constr',...
                'SubproblemAlgorithm', 'cg',...
                'TolProjCG', 1e-2,...
                'TolProjCGAbs', 1e-10);
        %                       'UseParallel','always',...
        elseif ( opt_option == 2 )
            options = optimset('Display','off',...
                'TolFun', tol_opt,...
                'MaxIter', 2000,...
                'Algorithm', 'trust-region-reflective',...
                'Hessian', 'off',...
                'MaxPCGIter', max(1,floor(size(u0,1)*size(u0,2)/2)),...
                'PrecondBandWidth', 0,...
                'TolPCG', 1e-1);
        end
    end
%==========================================================================
    
    warning off all
    t = [];
    x = [];
    u = [];
    comp_time = []; % computation time

    % Start of the NMPC iteration
    ti = 0;

    xx=[];

    th = 0;
    dth = 0;
    params.th = th;
    params.dth = dth;    
    Tc = params.Tc;
                

    tt=[];
%==========================================================================        
    [t0, x0] = measureInitialValue ( tmeasure, xmeasure );
    t_Start = tic;
    [u_new, V_current, exitflag, output] = solveOptimalControlProblem ...
        (@runningcosts, @terminalcosts, @constraints, ...
        @terminalconstraints, @linearconstraints, @system, @cov_propagation, ...
        N, t0, x0, u0, Th, ...
        sig, beta, params, ...
        atol_ode_sim, rtol_ode_sim, tol_opt, options, type);
    t_Elapsed = toc( t_Start );

    printSolution(@system, @printHeader, @printClosedloopData, ...
                      @plotTrajectories, ti, Th, t0, x0, u_new, params, ...
                      atol_ode_sim, rtol_ode_sim, type, iprint, ...
                      exitflag, output, t_Elapsed);

    writematrix(u_new,'data/u2.csv','WriteMode','append')
    
    

    out = u_new;
end

%==========================================================================
function [t0, x0] = measureInitialValue ( tmeasure, xmeasure )
    t0 = tmeasure;
    x0 = xmeasure;
end
%==========================================================================
function u0 = shiftHorizon(u)
    u0 = [u(:,2:size(u,2)) u(:,size(u,2))];
end
%==========================================================================
function [u, V, exitflag, output] = solveOptimalControlProblem ...
    (runningcosts, terminalcosts, constraints, terminalconstraints, ...
    linearconstraints, system, cov_propagation, N, t0, x0, u0, Th, sig, beta, params, ...
    atol_ode_sim, rtol_ode_sim, tol_opt, options, type)
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, Th, t0, x0, u0, ...
                                atol_ode_sim, rtol_ode_sim, type, sig, params);

    % Set control and linear bounds
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [];
    ub = [];
    for k=1:N
        [Anew, bnew, Aeqnew, beqnew, lbnew, ubnew] = ...
               linearconstraints(t0+k*Th,x(k,:),u0(:,k));
        A = blkdiag(A,Anew);
        b = [b, bnew];
        Aeq = blkdiag(Aeq,Aeqnew);
        beq = [beq, beqnew];
        lb = [lb, lbnew];
        ub = [ub, ubnew];
    end
    
    % Solve optimization problem
    % tic
    [u, V, exitflag, output] = fmincon(@(u) costfunction(runningcosts, ...
        terminalcosts, system, N, Th, t0, x0, ...
        u, atol_ode_sim, rtol_ode_sim, type, sig, params), u0, A, b, Aeq, beq, lb, ...
        ub, @(u) nonlinearconstraints(constraints, terminalconstraints, ...
        system, cov_propagation, N, Th, t0, x0, u, sig, beta, params, ...
        atol_ode_sim, rtol_ode_sim, type), options);
    % toc
end
%==========================================================================
function cost = costfunction(runningcosts, terminalcosts, system, ...
                    N, Th, t0, x0, u, ...
                    atol_ode_sim, rtol_ode_sim, type, sig, params)
    cost = 0;
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, Th, t0, x0, u, ...
                                atol_ode_sim, rtol_ode_sim, type, sig, params);
    for k=1:N
        cost = cost+runningcosts(t0+k*Th, x(k,:), u(:,k));
    end
    cost = cost+terminalcosts(t0+(N+1)*Th, x(N+1,:));
end
%==========================================================================
function [c,ceq] = nonlinearconstraints(constraints, ...
    terminalconstraints, system, cov_propagation, ...
    N, Th, t0, x0, u, sig, beta, params, atol_ode_sim, rtol_ode_sim, type)
    x = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, N, Th, t0, x0, u, ...
                                atol_ode_sim, rtol_ode_sim, type, sig, params);
    c = [];
    ceq = [];
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%% constraint tightening computation (& constraint generation)%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%==========================================================================
%       SMPC settings
%==========================================================================            
    
    % compute covariance matrix propagation
    % sigma_e = cov_propagation(N, sig,params);    
    % g = [-1;0];              % constraint: g*x < h -> x1 < x1_limit
    g1 = [0;0;0;0];              % constraint: g*x < h -> x1 < x1_limit
    g2 = [0;0;0;0];              % constraint: g*x < h -> x1 < x1_limit
    
    g1(params.state) = -1; % min
    g2(params.state) = 1;  % max

    K = [0,0,0,0];
    
    for k=1:N               % k=1 refers to the initial state, so (technically) no state constraint necessary, but k=1 necessary for input constraint

        % gamma1 = sqrt(2*g1'*sigma_e(:,:,k)*g1)*erfinv(2*beta-1);                   % constraint tightening
        % gamma2 = sqrt(2*g2'*sigma_e(:,:,k)*g2)*erfinv(2*beta-1);                   % constraint tightening
        [cnew, ceqnew] = constraints(t0+k*Th,x(k,:),u(:,k), 0, 0, K, params);   % generate constraints

        c = [c cnew];
        ceq = [ceq ceqnew];
    end
    [cnew, ceqnew] = terminalconstraints(t0+(N+1)*Th,x(N+1,:),params);
    c = [c cnew];
    ceq = [ceq ceqnew];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%% end of constraint tightening %%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
%==========================================================================
function x = computeOpenloopSolution(system, N, Th, t0, x0, u, ...
                                     atol_ode_sim, rtol_ode_sim, type, sig, params)
    x(1,:) = x0;

    uncertainty_flag = 0;
    for k=1:N
        x(k+1,:) = dynamic(system, Th, t0, x(k,:), u(:,k), ...
                             atol_ode_sim, rtol_ode_sim, type, uncertainty_flag, sig, params);
    end
end
%==========================================================================
function [x, t_intermediate, x_intermediate] = dynamic(system, Th, t0, ...
             x0, u, atol_ode, rtol_ode, type, apply_flag, sig, params)
    if ( strcmp(type, 'difference equation') )
        x = system(t0, x0, u, Th, apply_flag, sig, params);
        x_intermediate = [x0; x];
        t_intermediate = [t0, t0+Th];
    elseif ( strcmp(type, 'differential equation') )
        options = odeset('AbsTol', atol_ode, 'RelTol', rtol_ode);
        [t_intermediate,x_intermediate] = ode45(system, ...
            [t0, t0+Th], x0, options, u, apply_flag, sig, params);
        x = x_intermediate(size(x_intermediate,1),:);
    end
end
%==========================================================================

%==========================================================================
% NMPC functions
%==========================================================================
function cost = runningcosts(t, x, u)


    Q = [1   0  0   0;
         0   0  0   0;
         0   0  0  0;
         0   0  0  0];
    R = 0.;
    xd = [-0.58,0,0,0];
    cost = 1.*(x-xd)*Q*(x-xd)' + u(1)*R*u(1)';
    
% next two lines only necessary for slack variable
%     lambda = 5000;  
%     cost = x*Q*x' + u(1)*R*u(1)' + lambda*u(2);

end
%==========================================================================
function cost = terminalcosts(t, x)

    cost = 0.0;
end
%==========================================================================
function [c,ceq] = constraints(t, x, u, gamma1, gamma2, K, params)

    x1_limit = params.x1_limit;                       % get x1 constraint    
    ulim = params.ulim;                       % get x1 constraint    
    c   = [];
    K = [0,0,0,0];
   
    % Control limit
    c(end+1) = (u(1) - K*[x(1); x(2); x(3); x(4)]) - ulim;
    c(end+1) = -(u(1) - K*[x(1); x(2); x(3); x(4)]) - ulim;

%==========================================================================
%       SMPC settings
%==========================================================================            
    % Chance Constraint
    % gamma1 - min , gamma2 - max
    % c(end+1) = -x(params.state) + x1_limit(1) + gamma1;            % g'*x-2.8 = [1 0]*[x(1);x(2)]-2.8
    % c(end+1) =  x(params.state) - x1_limit(2)  + gamma2;            % g'*x-2.8 = [1 0]*[x(1);x(2)]-2.8

    ceq = [];

end
%==========================================================================
function [c,ceq] = terminalconstraints(t,x,params)
    c   = [];
    ceq   = [];
end
%==========================================================================
function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb = [];
    ub = [];
end
%==========================================================================
function y = system(t, x, u, Th, apply_flag, sig, params)
    % apply_flag: 1) noise is applied for real system; 0) no noise for prediction
    A = params.sysA;
    B = params.sysB;
    % K = [0.8151    2.7097   11.8373    3.7545];    
    K = [0,0,0,0];


    y = A*x'+B*(u(1,1) - K*[x(1); x(2); x(3); x(4)]);
    
    if apply_flag == 1
        D = [0 0 0 0;
             0 0 0 0;
             0 0 1 0;
             0 0 0 0];
        w = [0;0;0;0];
        w(1) = normrnd(0,sig);
        w(2) = normrnd(0,sig);
        w(3) = normrnd(0,sig);
        w(4) = normrnd(0,sig);
        y = y + D*w;
    end
    y = y';    
end
%==========================================================================
function sigma_e = cov_propagation(N, sig, params)
    w_cov = [sig^2 0 0 0;
             0 sig^2 0 0;
             0 0 sig^2 0;
             0 0 0 sig^2];
    A = params.sysA;
    B = params.sysB;
%==========================================================================
%       SMPC settings
%==========================================================================            
    
    K = [0 0 0 0];    
    D = [0 0 0 0;
         0 0 0 0;
         0 0 1 0;
         0 0 0 0];

    phi = A-B*K;

    sigma_e = zeros(4,4,N);

    for i = 2:N
        sigma_e(:,:,i) = phi*sigma_e(:,:,i-1)*phi' + D*w_cov*D';
    end
    
end
%==========================================================================
function printHeader()
    fprintf('   k  |       u(k)         x(1)        x(2)       Time   | Solver messages\n');
    fprintf('--------------------------------------------------------------------------\n');
end
%==========================================================================
function printClosedloopData(ti, u, x, t_Elapsed)
    fprintf(' %3d  | %+11.3f %+11.2f %+11.2f      %+6.3f  |', ti, ...
            u(1,1), x(1), x(2), t_Elapsed);
end
%==========================================================================
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function printSolution(system, printHeader, printClosedloopData, ...
             plotTrajectories, ti, Th, t0, x0, u, params, ...
             atol_ode, rtol_ode, type, iprint, exitflag, output, t_Elapsed)
    if (ti == 0)
        printHeader();
    end
    printClosedloopData(ti, u, x0, t_Elapsed);
    switch exitflag
        case -2
        if ( iprint >= 1 && iprint < 10 )
            fprintf(' Error F\n');
        elseif ( iprint >= 10 )
            fprintf(' Error: No feasible point was found\n')
        end
        case -1
        if ( iprint >= 1 && iprint < 10 )
            fprintf(' Error OT\n');
        elseif ( iprint >= 10 )
            fprintf([' Error: The output function terminated the',...
                     ' algorithm\n'])
        end
        case 0
        if ( iprint == 1 )
            fprintf('\n');
        elseif ( iprint >= 2 && iprint < 10 )
            fprintf(' Warning IT\n');
        elseif ( iprint >= 10 )
            fprintf([' Warning: Number of iterations exceeded',...
                     ' options.MaxIter or number of function',...
                     ' evaluations exceeded options.FunEvals\n'])
        end
        case 1
        if ( iprint == 1 )
            fprintf('\n');
        elseif ( iprint >= 2 && iprint < 10 )
            fprintf(' \n');
        elseif ( iprint >= 10 )
            fprintf([' [Y] First-order optimality measure was less',...
                     ' than options.TolFun, and maximum constraint',...
                     ' violation was less than options.TolCon\n'])
        end
        case 2
        if ( iprint == 1 )
            fprintf('\n');
        elseif ( iprint >= 2 && iprint < 10 )
            fprintf(' Warning TX\n');
        elseif ( iprint >= 10 )
            fprintf(' [Y] Warning: Change in x was less than options.TolX\n')
        end
        case 3
        if ( iprint == 1 )
            fprintf('\n');
        elseif ( iprint >= 2 && iprint < 10 )
            fprintf(' Warning TJ\n');
        elseif ( iprint >= 10 )
            fprintf(['  [Y] Warning: Change in the objective function',...
                     ' value was less than options.TolFun\n'])
        end
        case 4
        if ( iprint == 1 )
            fprintf('\n');
        elseif ( iprint >= 2 && iprint < 10 )
            fprintf(' Warning S\n');
        elseif ( iprint >= 10 )
            fprintf(['  [Y] Warning: Magnitude of the search direction',...
                     ' was less than 2*options.TolX and constraint',...
                     ' violation was less than options.TolCon\n'])
        end
        case 5
        if ( iprint == 1 )
            fprintf('\n');
        elseif ( iprint >= 2 && iprint < 10 )
            fprintf(' Warning D\n');
        elseif ( iprint >= 10 )
            fprintf(['  [Y] Warning: Magnitude of directional derivative',...
                     ' in search direction was less than',...
                     ' 2*options.TolFun and maximum constraint',...
                     ' violation was less than options.TolCon\n'])
        end
    end
    % if ( iprint >= 5 )
    %     plotTrajectories(@dynamic, system, Th, t0, x0, u, atol_ode, rtol_ode, type, params)
    % end
end
%==========================================================================

