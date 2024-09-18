function sol = dual_structural_controller(N,N_s,n_m,n_x,n_u,L,A,B,Ts,varW,ivarW,Q,R,my_mvnpdf)

import casadi.*


X{1} =  SX.sym('X0',n_x,1); %x(:,t);
P_M{1} = SX.sym('P_M0',n_m,1); %p_M(:,t);
p_cost{1} = 1;
MU_gamma_M{1} = SX.sym('MU_gamma_M', n_m,1); %mu_gamma_M(:,t);
VAR_gamma_M{1}= SX.sym('VAR_gamma_M', n_m,1); %var_gamma_M(:,t);
ref = SX.sym('ref',1,N);

for i=1:L
    w{i} = SX.sym('w', n_x,(N_s*n_m)^i); %mW + chol(varW)*randn(n_x,(N_s*n_m)^i);
    gamma_M{i} = SX.sym('gamma_M', 1,(N_s*n_m)^i); %randn(1,(N_s*n_m)^i);
    X{i+1} = SX.sym('X',n_x,(N_s*n_m)^i);
    UL{i} = SX.sym('UL',n_u,(N_s*n_m)^(i-1));
    P_M{i+1} = SX.sym('P_M',n_m*(N_s*n_m)^i,1);
    p_cost{i+1} = SX.sym('p_cost',(N_s*n_m)^i,1);
    MU_gamma_M{i+1} = SX.sym('MU_gamma_M', n_m*(N_s*n_m)^i,1);
    VAR_gamma_M{i+1} = SX.sym('VAR_gamma_M', n_m*(N_s*n_m)^i,1);
end

wunwrap = [];
gammaunwrap = [];
for i=1:L
    wunwrap = [wunwrap, w{i}(:)'];
    gammaunwrap = [gammaunwrap,gamma_M{i}(:)'];
end

param = [ref, X{1}(:)', P_M{1}(:)', MU_gamma_M{1}(:)', VAR_gamma_M{1}(:)',wunwrap,gammaunwrap];

for i=1:N-L
    mX{i} = SX.sym('mX',n_x,n_m*(N_s*n_m)^L);
    U{i} = SX.sym('mX',n_u,(N_s*n_m)^L);
end

%lookahead phase
for k=1:L

    j{k} = (1:(N_s*n_m)^(k-1));

    for jk=1:length(j{k})
        for m=1:n_m
            for l=1:N_s
                idx = N_s*(m-1) + (jk-1)*N_s*n_m + l;
                
                % sampling
                gamma_M{k}(:,idx) = MU_gamma_M{k}(m + (jk-1)*n_m,:) + gamma_M{k}(:,idx)*chol(VAR_gamma_M{k}(m + (jk-1)*n_m,:));
                
                % forward dynamics
                X{k+1}(:,idx) = (A*Ts + eye(n_x))*X{k}(:,jk) + Ts*B(:,m)*gamma_M{k}(:,idx)*UL{k}(:,jk) + Ts*w{k}(:,idx);
                
                % distribution updates
                normalization = 0;
                for p=1:n_m
                    x_eval = X{k+1}(:,idx) - (A*Ts + eye(n_x))*X{k}(:,jk);

                    VAR_gamma_M{k+1}(p + (idx-1)*n_m,:) = 1./( 1./VAR_gamma_M{k}(p + (jk-1)*n_m,:) + (Ts*B(:,p)*UL{k}(:,jk))'*ivarW*(Ts*B(:,p)*UL{k}(:,jk)));
                    MU_gamma_M{k+1}(p + (idx-1)*n_m,:) = VAR_gamma_M{k+1}(p + (idx-1)*n_m,:)*( MU_gamma_M{k}(p + (jk-1)*n_m,:)./VAR_gamma_M{k}(p + (jk-1)*n_m,:) + (Ts*B(:,p)*UL{k}(:,jk))'*ivarW*x_eval );

                    mu =  Ts*B(:,p)*UL{k}(:,jk)*MU_gamma_M{k}(p + (jk-1)*n_m,:);
                    var = (Ts*B(:,p)*UL{k}(:,jk))*(VAR_gamma_M{k}(p + (jk-1)*n_m,:))*(Ts*B(:,p)*UL{k}(:,jk))' + varW;

                    P_M{k+1}(p + (idx-1)*n_m,:) = my_mvnpdf(x_eval,mu,var)*P_M{k}(p + (jk-1)*n_m,:);
                    normalization = normalization + P_M{k+1}(p + (idx-1)*n_m,:);

                end
                P_M{k+1}(((1 + (idx-1)*n_m):(n_m + (idx-1)*n_m) ),:) = P_M{k+1}(((1 + (idx-1)*n_m):(n_m + (idx-1)*n_m) ),:)/normalization;
                p_cost{k+1}(idx,:) = P_M{k}(m + (jk-1)*n_m,:)*p_cost{k}(jk,:); % cost weight
            end
        end
    end

end

% exploitation phase
mX{1} = kron(X{L+1},ones(1,n_m));
j{L+1} = (1:(N_s*n_m)^L);
for k=1:N-L-1
    for jk=1:length(j{L+1})
        for m=1:n_m
            mX{k+1}(:,m + (jk-1)*n_m) = (A*Ts + eye(n_x))*mX{k}(:,m + (jk-1)*n_m) + Ts*B(:,m)*U{k}(:,jk)*MU_gamma_M{L+1}(m + (jk-1)*n_m,:);
        end
    end
end

% cost function
cost = 0;
for k=1:L
    cost1=0;
    for jk=1:length(j{k})
        cost1 = cost1 + p_cost{k}(jk,:)*((X{k}(:,jk) - [0;0;0;0])'*Q*(X{k}(:,jk) - [0;0;0;0]) + UL{k}(:,jk)'*R*UL{k}(:,jk));
    end
    cost1 = cost1/((N_s)^(k-1));
    cost = cost + cost1;
end

for k=1:N-L-1
    cost1=0;
    for jk=1:length(j{L+1})
        for m=1:n_m
            cost1 = cost1 + p_cost{L+1}(jk,:)*(P_M{L+1}(m + (jk-1)*n_m,:)*(  (mX{k}(:,m + (jk-1)*n_m) - [0;0;0;0])'*Q*(mX{k}(:,m + (jk-1)*n_m) - [0;0;0;0]) + U{k}(:,jk)'*R*U{k}(:,jk)  ) );
        end
    end
    cost1 = cost1/((N_s)^(L));
    cost = cost + cost1;
end

cost1=0;
for jk=1:length(j{L+1})
    for m=1:n_m
        cost1 = cost1 + p_cost{L+1}(jk,:)*(P_M{L+1}(m + (jk-1)*n_m,:)*(  (mX{N-L}(:,m + (jk-1)*n_m) - [0;0;0;0])'*Q*(mX{N-L}(:,m + (jk-1)*n_m) - [0;0;0;0])  + U{N-L}(:,jk)'*R*U{k}(:,jk) ) );
    end
end
cost1 = cost1/((N_s)^(L));
cost = cost + cost1;

vars = [];
for i=1:L
    vars=[vars,UL{i}(:)'];
end
for i=1:N-L
    vars=[vars,U{i}(:)'];
end

% vars'
% cost
% disp('dgffdgdfgdg')
% param

nlp = struct('x',vars', 'f',cost,'p',param);
%opt = struct('print_time',false,'ipopt',struct('print_level',0,'file_print_level',0));
sol = nlpsol('S', 'ipopt', nlp);% nlp,opt);

end

