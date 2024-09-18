function [u,p_M, mu_gamma_M,var_gamma_M] = run_stsmpc(A , B , Ts , Q , R , n_x , n_u , n_m , true_gamma , mW ,  varW , ivarW , mu_gamma_M , var_gamma_M , p_M ,  x ,  T , N , k_fault , delay , ref,  N_s , L , w_ , simulation , sol, t, u)



        if t>1

            t=t-1;
            
            normlz = 0;
            for model=1:n_m
                % disp('x
                % t+1')
                % disp(x(:,t+1))
                x0 = x(:,t+1) - (A*Ts + eye(n_x))*x(:,t);
    
                var_gamma_M(model,t+1) = 1./( 1./var_gamma_M(model,t) + (Ts*B(:,model)*u(:,t))'*ivarW*(Ts*B(:,model)*u(:,t)) );
                % mu_gamma_M(model,t+1) = var_gamma_M(model,t+1)*( mu_gamma_M(model,t)./var_gamma_M(model,t) + (Ts*B(:,model)*u(:,t))'*ivarW*x0 );
                mu_gamma_M(model,t+1) = 1;
    
                mu = Ts*B(:,model)*u(:,t)*mu_gamma_M(model,t);
                var = (Ts*B(:,model)*u(:,t))*(var_gamma_M(model,t))*(Ts*B(:,model)*u(:,t))' + varW;

                disp(x0)

                p_M(model,t+1) = mvnpdf(x0,mu,var)*p_M(model,t);
                normlz = normlz + p_M(model,t+1);
            end
            p_M(:,t+1) = p_M(:,t+1)/normlz;
    
            p_M(p_M > 0.99) = 0.99;
            p_M(p_M < 0.01) = 0.01;

            t=t+1;
        end


    for i=1:L
        gamma_M{i} = randn(1,(N_s*n_m)^i);
        w{i} = mW + chol(varW)*randn(n_x,(N_s*n_m)^i);
    end
    wunwrap = [];
    gammaunwrap = [];
    for i=1:L
        wunwrap = [wunwrap, w{i}(:)'];
        gammaunwrap = [gammaunwrap,gamma_M{i}(:)'];
    end
    
    param = [ref(t:t+N-1)', x(:,t)', p_M(:,t)', mu_gamma_M(:,t)',var_gamma_M(:,t)',wunwrap,gammaunwrap];  
    r = sol('x0',0,'lbx',-0.2,'ubx',+0.2,'p',param);
    
    x_opt = r.x;

    u(:,t) = full(x_opt(1:n_u));        


end

