
clc
clear all

tempsimul = 1000;

accel_x_colum = 1;
accel_y_colum = 2;
accel_z_colum = 3;
gyro_x_colum  = 4;
gyro_y_colum  = 5;
gyro_z_colum  = 6;

accel_x_offset = 20;
accel_y_offset = 40;
accel_z_offset = 70;

conversion_factor = 0.0005;

accel_g_x = zeros(tempsimul,1);
accel_g_y = zeros(tempsimul,1);
accel_g_z = zeros(tempsimul,1);

gyro_x = zeros(tempsimul,1);
gyro_y = zeros(tempsimul,1);
gyro_z = zeros(tempsimul,1);

angle_xz = zeros(tempsimul,1);
angle_yz = zeros(tempsimul,1);
angle_xy = zeros(tempsimul,1);

%gyro_accel_raw = load('./accel_gyro_3_axis_1K.m');
%gyro_accel_raw = load('./gyro_accel_3_axis_1k.m');
%gyro_accel_raw = load('./../gyro_accel_3_axis_1k_3.m');
%gyro_accel_raw = load('./../45_degrees_700_samples.m');
gyro_accel_raw = load('./../samples/teste.m');
%gyro_accel_raw = load('./../from_mnus_90_to_90.m');




for i=1:tempsimul
  gyro_x(i) = (gyro_accel_raw(i,[gyro_x_colum]));
  gyro_y(i) = (gyro_accel_raw(i,[gyro_y_colum]));
  gyro_z(i) = (gyro_accel_raw(i,[gyro_z_colum]));  
end

for i=1:tempsimul
  angle_xz(i) = gyro_accel_raw(i,[accel_x_colum]);
  angle_yz(i) = gyro_accel_raw(i,[accel_y_colum]);
  angle_xy(i) = gyro_accel_raw(i,[accel_z_colum]);
end

x_updated = zeros(6,tempsimul);
x_predicted = zeros(6,tempsimul);
z = zeros(3,tempsimul);
u = zeros(3,tempsimul);

z([1],:) = angle_xz';
z([2],:) = angle_yz';
z([3],:) = angle_xy';

u([1],:) = gyro_x;
u([2],:) = gyro_y;
u([3],:) = gyro_z;


x_predicted([1],1) = 0;
x_predicted([2],1) = 0;
x_predicted([3],1) = 0;
x_updated([1],2) = 165; %initial angle
x_updated([2],2) = 27; %initial angle
x_updated([3],2) = 57; %initial angle

p_updated = zeros(6,6);
p_predicted = zeros(6,6);

delta_time = 0.01;

A = [ 1 -delta_time   0       0         0       0;
      0       1       0       0         0       0;
      0       0       1 -delta_time     0       0;
      0       0       0       1         0       0;
      0       0       0       0         1 -delta_time;
      0       0       0       0         0       1];
   
B = [delta_time    0        0;
       0           0        0;
       0      delta_time    0;
       0           0        0;
       0           0   delta_time;
       0           0        0];
    
H = [ 1   0   0   0   0   0;
      0   0   1   0   0   0;
      0   0   0   0   1   0];

R = 5;

Q = [0.5  0   0   0   0   0;
      0  0.5  0   0   0   0;
      0   0  0.5  0   0   0;
      0   0   0  0.5  0   0;
      0   0   0   0  0.5  0;
      0   0   0   0   0  0.5];



for i=2:tempsimul
  t(i)=i;
  #####################################################################################
  %Time Update (“Predict”) 
  % Project the state ahead
  x_predicted(:,i)  = A * x_updated(:,i-1) +  B * u(:,i-1);   #u(i) == giro_rw_data(i)
  % Project the error covariance ahead
  p_predicted  = A * p_updated * A' + Q;  
  #####################################################################################
  
  
  #####################################################################################
  %Measurement Update (“Correct” 
  %Compute the Kalman gain
  kalman_gain  = p_predicted*H' * inv(H*p_predicted*H' + R);
  
  %Update estimate with measurement zk
  x_updated(:,i) = x_predicted(:,i) + kalman_gain*(z(:,i) - H * x_predicted(:,i));
 
  %Update the error covariance
  p_updated  = (eye(6,6) - kalman_gain * H) * p_predicted; 
  #####################################################################################
end

plot(t,x_updated(1,:),'b',t,0.05*u([1],:),'r',t,z([1],:),'g');
%plot(t,x_updated(3,:),'b',t,0.05*u([2],:),'r',t,z([2],:),'g');
%plot(t,x_updated(5,:),'b',t,0.05*u([3],:),'r',t,z([3],:),'g');
axis ([0, 1000, -180, 180], "square") ;