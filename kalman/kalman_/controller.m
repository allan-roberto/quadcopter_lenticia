
clc
clear all
delta_time = 0.01;

A = [ 1       0       0       0         0       0;
      0       1       0       0         0       0;
      0       0       1       0         0       0;
      0       0       0       1         0       0;
      0       0       0       0         1       0;
      0       0       0       0         0       1];
   
B = [delta_time    0        0;
       0           0        0;
       0      delta_time    0;
       0           0        0;
       0           0   delta_time;
       0           0        0];
    
C = [ 1   0   0   0   0   0;
      0   0   1   0   0   0;
      0   0   0   0   1   0];
      
D = [ 0   0   0;
      0   0   0;
      0   0   0];

     
states = {'x' 'x_dot' 'phi' 'phi_dot' 'y' 'y_dot'};
inputs = {'u1' 'u2' 'u3'};
outputs = {'x' 'phi' 'y'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
poles = eig(A);
co = ctrb(A,B);
Controllability = rank(co);
p = 2;
Q = p*C'*C;
R = [ 1 0 0;
      0 1 0;
      0 0 1];
[K] = lqr(A,B,Q,R)