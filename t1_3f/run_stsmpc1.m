function [u,p_M, mu_gamma_M,var_gamma_M] = run_stsmpc1(A , B , Ts , Q , R , n_x , n_u , n_m , true_gamma , mW ,  varW , ivarW , mu_gamma_M , var_gamma_M , p_M ,  x ,  T , N , k_fault , delay , ref,  N_s , L , w_ , simulation , sol, t)


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

