function ay = pd_y(y,vy,yi)

kp_y = 1.0;
kd_y = 0.1;

ay = kp_y*(yi-y) + kd_y*(-vy);

end