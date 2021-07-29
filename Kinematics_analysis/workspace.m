clear all; clc; 
L1 = 10.03; L2 = 4.246; L3 = 2.53012; alpha = 0;
theta1 =pi/4;
   for theta2 = 0:pi/30:2*pi
       for theta3 = 0:pi/30:2*pi
           x = ((-L3*sin(alpha)-L2)*cos(theta2)-sin(theta2)*sin(theta3)*L3*cos(alpha))*sin(theta1)+cos(theta1)*(cos(theta3)*L3*cos(alpha)+L1);
           y = ((L3*sin(alpha)+L2)*cos(theta2)+sin(theta2)*sin(theta3)*L3*cos(alpha))*cos(theta1)+sin(theta1)*(cos(theta3)*L3*cos(alpha)+L1);
           z = (L3*sin(alpha)+L2)*sin(theta2)-L3*cos(alpha)*sin(theta3)*cos(theta2);
           
           plot3(x,z,y,'-o','Color','b','MarkerSize',2,'MarkerFaceColor','#D9FFFF')
           hold on
       end
   end
