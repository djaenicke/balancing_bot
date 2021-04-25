dt = 0.005;

% State vector
% [phi; phi_dot_bias]
x = [0; 0];

% State-transition model
F = [1 -dt; 0 1];

% Control-input model
B = [dt; 0];

% Observation model
H = [1 0];

% State estimate covariance matrix
P = [1e3 0; 0 1e3];

% Covariance of the process noise
Q_angle = 0.0008;
Q_bias = 0.01;
Q = [Q_angle 0; 0 Q_bias];

% Covariance of the observation noise
R = 0.5;

% Gain
K = [0 0; 0 0];

% Filtered output
phi_output = zeros(size(phi, 1), 1);
phi_dot_bias_output = zeros(size(phi, 1), 1);

for i=1:size(phi, 1)
  % Prediction Step
  x = (F * x) + (B * phi_dot(i));  % a priori state estimate
  P = F * P * F' + Q; % a priori state estimate covariance matrix
  
  % Update Step
  K = P * H' * ((H * P * H') + R)^-1;
  P = (eye(2) - (K * H)) * P;
  x = x + K * (phi(i) - (H * F * x));
  
  % Grab the estimated state
  phi_output(i) = x(1);
  phi_dot_bias_output(i) = x(2);
end

figure(2);
plot(phi);
hold on;
plot(phi_output);
legend('phi', 'phi filtered');
