function  Blender(TOUT,YOUT)
Time=TOUT(:);
x=YOUT(:,10);
y=YOUT(:,11);
z=YOUT(:,12);
Phi=YOUT(:,2)*180/pi;
Psi=YOUT(:,3)*180/pi;
Theta=YOUT(:,1)*180/pi;
save('T.mat','Time','Theta' ,'Phi' ,'Psi' , 'x' ,'y' ,'z');
end

