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

# xt = 0;

qd_i = matlab.double([[q[0] - xt], [q[1]], [q[2]], [q[3]]])

qd_i2 = matlab.double(q2)


qd_des = matlab.double([[0.0], [0.0], [0.0], [0.0]])

t = 1

u = matlab.double([])

print(mu_gamma_M)

[ud,p_M, mu_gamma_M,var_gamma_M] = eng.run_stsmpc(A , B , Ts , Q , R , n_x , n_u , n_m , true_gamma , mW ,  varW , ivarW , mu_gamma_M , var_gamma_M , p_M ,  qd_i2,  T , N , k_fault , delay , ref,  N_s , L , w_ , simulation , sol, t, u, nargout=4)



print(ud)


q3 = np.array([q2.T[0], q2.T[0]])

# print(q2.T[0])

t = 2

qd_i = np.array([[0.0, 0.0, 0.0, 0.0],[0.1, 0.0, 0.0, 0.0]])



q3m = matlab.double(q3.T)

[ud2,p_M,mu_gamma_M,var_gamma_M] = eng.run_stsmpc(A , B , Ts , Q , R , n_x , n_u , n_m , true_gamma , mW ,  varW , ivarW , mu_gamma_M , var_gamma_M , p_M ,  q3m ,  T , N , k_fault , delay , ref,  N_s , L , w_ , simulation , sol, t, ud, nargout=4)


# size of q3.T = 2


print('q3')
print(q3)

print('q2.T[0]')
print(q2.T[0])

q3 = np.vstack((q3, q2.T[0]))

print('q3')
print(q3)
    
# q3 = np.array([q3, q2.T[0]])

# print(q2.T[0])

t = 2

qd_i = np.array([[0.0, 0.0, 0.0, 0.0],[0.1, 0.0, 0.0, 0.0]])



q3m = matlab.double(q3.T)

[ud2,p_M,mu_gamma_M,var_gamma_M] = eng.run_stsmpc(A , B , Ts , Q , R , n_x , n_u , n_m , true_gamma , mW ,  varW , ivarW , mu_gamma_M , var_gamma_M , p_M ,  q3m ,  T , N , k_fault , delay , ref,  N_s , L , w_ , simulation , sol, t, ud, nargout=4)


# l1 = [1, 2, 3]
# print(matlab.double(np.array(l1,[])))


