
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

%gyro_accel_raw = load('./accel_gyro_3_axis_1K.m');
%gyro_accel_raw = load('./gyro_accel_3_axis_1k.m');
%gyro_accel_raw = load('./../gyro_accel_3_axis_1k_3.m');
%gyro_accel_raw = load('./../45_degrees_700_samples.m');
gyro_accel_raw = load('./../samples/pitch_shaking_10ms_1000_samples_3.m');
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

x_updated = zeros(4,tempsimul);
x_predicted = zeros(4,tempsimul);
z = zeros(2,tempsimul);
u = zeros(2,tempsimul);

z([1],:) = angle_xz';
z([2],:) = angle_yz';

u([1],:) = gyro_y;
u([2],:) = gyro_x;


x_predicted([1],1) = 0;
x_updated([2],1) = angle_xz(1); %initial angle

p_updated = zeros(4,4);
p_predicted = zeros(4,4);

delta_time = 0.01;

A = [ 1 -delta_time   0       0; 
      0       1       0       0;
      0       0       1 -delta_time;
      0       0       0       1];
   
B = [delta_time    0;
       0           0;
       0      delta_time;
       0           0];
    
H = [ 1   0   0   0
      0   0   1   0];

R = 200;

Q = [ 1   0   0   1;
      0   1   0   0;
      0   0   1   1;
      0   0   0   1];



for i=2:tempsimul
  t(i)=i;
  #####################################################################################
  %Time Update (“Predict”) 
  % Project the state ahead
  x_predicted(:,i)  = A * x_updated(:,i-1) +  B * u(:,i-1);   #u(i) == giro_rw_data(i)
  % Project the error covariance ahead
  p_predicted  = A * p_updated +  A' + Q;  
  #####################################################################################
  
  
  #####################################################################################
  %Measurement Update (“Correct” 
  %Compute the Kalman gain
  kalman_gain  = p_predicted*H' * inv(H*p_predicted*H' + R);
  
  %Update estimate with measurement zk
  x_updated(:,i)    = x_predicted(:,i) + kalman_gain*(z(:,i) - H * x_predicted(:,i));
 
  %Update the error covariance
  p_updated  = (eye(4,4) - kalman_gain * H) * p_predicted; 
  #####################################################################################
end

%plot(t,x_predicted(1,:),'b',t,z,'r',t,gyro_x,'g',t,gyro_y,'r',t,100*accel_g_x,'d',t,100*accel_g_z,'p')
plot(t,x_updated(1,:),'b',t,z(1,:),'g',t,0.05 * u(1,:),'r');
axis ([0, 1000, -90, 90], "square") ;
%plot(t,z,'r')