function [u,p_M, mu_gamma_M,var_gamma_M] = run_stsmpc2(A , B , Ts , Q , R , n_x , n_u , n_m , true_gamma , mW ,  varW , ivarW , mu_gamma_M , var_gamma_M , p_M ,  x ,  T , N , k_fault , delay , ref,  N_s , L , w_ , simulation , sol, t, u)




        normlz = 0;
        for model=1:n_m
            x0 = x(:,t+1) - (A*Ts + eye(n_x))*x(:,t);

            var_gamma_M(model,t+1) = 1./( 1./var_gamma_M(model,t) + (Ts*B(:,model)*u(:,t))'*ivarW*(Ts*B(:,model)*u(:,t)) );
            % mu_gamma_M(model,t+1) = var_gamma_M(model,t+1)*( mu_gamma_M(model,t)./var_gamma_M(model,t) + (Ts*B(:,model)*u(:,t))'*ivarW*x0 );
            mu_gamma_M(model,t+1) = 1;

            mu = Ts*B(:,model)*u(:,t)*mu_gamma_M(model,t);
            var = (Ts*B(:,model)*u(:,t))*(var_gamma_M(model,t))*(Ts*B(:,model)*u(:,t))' + varW;

            p_M(model,t+1) = mvnpdf(x0,mu,var)*p_M(model,t);
            normlz = normlz + p_M(model,t+1);
        end
        p_M(:,t+1) = p_M(:,t+1)/normlz;

        p_M(p_M > 0.95) = 0.99;
        p_M(p_M < 0.05) = 0.01;



end

