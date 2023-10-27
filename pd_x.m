function ax = pd_x(x,vx,xi)

kp_x = 1.0;
kd_x = 0.1;

ax = kp_x*(xi-x) + kd_x*(-vx);

end