function [] = Animation(j, s, Dims, Obst_num, h1, Ts, r_min, r_min2, qo_i, qd_i, qd, qd_des, filename)
    % Plot Data
    cla
    if Dims == 2
        plot(qd_des(1), qd_des(2),'-p','MarkerFaceColor','red','MarkerSize',15)
        plot(qd_i(1),qd_i(2),'ro','MarkerFaceColor','black')
        for i = 1:Obst_num
            plot(qo_i(1,i),qo_i(2,i),'b*', qd(1,:),qd(2,:),'k--')
            viscircles([qo_i(1,i),qo_i(2,i)],r_min)
            viscircles([qo_i(1,i),qo_i(2,i)],r_min2,'LineStyle','--')
        end
    else
        view(23,30);
        plot3(qd_des(1), qd_des(2),qd_des(3),'-p','MarkerFaceColor','red','MarkerSize',15)
        plot3(qd_i(1),qd_i(2),qd_i(3),'ro','MarkerFaceColor','black')
        plot3(qd(1,:),qd(2,:),qd(3,:),'k--')
        for i = 1:Obst_num
            plot3(qo_i(1,i),qo_i(2,i),qo_i(3,i),'b*')
            sphere1 = surfl(s.x*r_min + qo_i(1,i), s.y*r_min + qo_i(2,i), s.z*r_min + qo_i(3,i)); 
            sphere2 = surfl(s.x*r_min2 + qo_i(1,i), s.y*r_min2 + qo_i(2,i), s.z*r_min2 + qo_i(3,i));
            set(sphere1, 'FaceColor', [1 0 0])
            set(sphere2, 'FaceAlpha', 0.1, 'FaceColor', [0 1 0])
        end
        xlim([-3,3]); 
        ylim([-3,3]); 
        zlim([-3,3]); 
    end
    drawnow;
    
   % GIF:
   frame = getframe(h1);
   im = frame2im(frame);
   [imind,cm] = rgb2ind(im,256);
   if j == 1
       imwrite(imind,cm,filename,'gif','DelayTime',Ts,'Loopcount',inf);
   elseif j == numel(j)
       imwrite(imind,cm,filename,'gif','DelayTime',Ts,'WriteMode','append');
   else
       imwrite(imind,cm,filename,'gif','DelayTime',Ts,'WriteMode','append');
   end  
end