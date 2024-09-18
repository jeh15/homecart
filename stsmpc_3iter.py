import numpy as np
import matlab.engine            #import the matlab engine
import copy


# -----------------------------------------------------------
# Matlab setup
print("Starting matlab engine...")
eng = matlab.engine.connect_matlab()
print("Matlab engine started.")

eng.addpath('/home/orl/repository/homecart/t1_3f')


# -----------------------------------------------------------





# -----------------------------------------------------------
# convert acc to joint traj
# [Th,Nodes,xd_lb,xd_ub] = eng.DevMPC6(nargout = 4)
[A , B , Ts , Q , R , n_x , n_u , n_m , true_gamma , mW ,  varW , ivarW , mu_gamma_M , var_gamma_M , p_M ,  x ,  T , N , k_fault , delay , ref,  N_s , L , w_ , simulation , sol] = eng.dev_stsmpc(nargout = 26)
# -----------------------------------------------------------

 
q = np.array([0.0, 0.0, 0.0, 0.0])
xt = 0;
q2 = np.array([[q[0] - xt], [q[1]], [q[2]], [q[3]]])
qd_i2 = matlab.double(q2)
t = 1
u = matlab.double([])

print('-------------------')
print('q1')
print(qd_i2)
print('-------------------')

[ud,p_M, mu_gamma_M,var_gamma_M] = eng.run_stsmpc(A , B , Ts , Q , R , n_x , n_u , n_m , true_gamma , mW ,  varW , ivarW , mu_gamma_M , var_gamma_M , p_M ,  qd_i2,  T , N , k_fault , delay , ref,  N_s , L , w_ , simulation , sol, t, u, nargout=4)


q3 = np.vstack((q2.T[0], q2.T[0]))
t = 2
q3m = matlab.double(q3.T)


[ud,p_M,mu_gamma_M,var_gamma_M] = eng.run_stsmpc(A , B , Ts , Q , R , n_x , n_u , n_m , true_gamma , mW ,  varW , ivarW , mu_gamma_M , var_gamma_M , p_M ,  q3m ,  T , N , k_fault , delay , ref,  N_s , L , w_ , simulation , sol, t, ud, nargout=4)



q3 = np.vstack((q3, q2.T[0]))
t = 3
q3m = matlab.double(q3.T)



[ud,p_M,mu_gamma_M,var_gamma_M] = eng.run_stsmpc(A , B , Ts , Q , R , n_x , n_u , n_m , true_gamma , mW ,  varW , ivarW , mu_gamma_M , var_gamma_M , p_M ,  q3m ,  T , N , k_fault , delay , ref,  N_s , L , w_ , simulation , sol, t, ud, nargout=4)



q3 = np.vstack((q3, q2.T[0]))
t = 4
q3m = matlab.double(q3.T)



[ud,p_M,mu_gamma_M,var_gamma_M] = eng.run_stsmpc(A , B , Ts , Q , R , n_x , n_u , n_m , true_gamma , mW ,  varW , ivarW , mu_gamma_M , var_gamma_M , p_M ,  q3m ,  T , N , k_fault , delay , ref,  N_s , L , w_ , simulation , sol, t, ud, nargout=4)

print('-------------------')
print('ud')
print(ud[0][-1])
print('-------------------')

