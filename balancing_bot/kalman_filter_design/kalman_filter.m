dt = 0.005;

% State vector
x = [0; phi_dot(1)];

% State-transition model
F = [1 dt; 0 1];

% Observation model
H = [1 0; 0 1];

% State estimate covariance matrix
P = [1e3 0; 0 1e3];

% Covariance of the process noise
Q = [0.001 0; 0 0.0005];

% Covariance of the observation noise
phi_variance = 0.5;
phi_dot_variance = 0.001;
R = [phi_variance 0; 0 phi_dot_variance];

% Gain
K = [0 0; 0 0];

% Filtered output
phi_output = zeros(size(phi, 1), 1);

for i=1:size(phi, 1)
  % Prediction Step
  x = F * x;  % a priori state estimate
  P = F * P * F' + Q; % a priori state estimate covariance matrix
  
  % Update Step
  K = P * H' * ((H * P * H') + R)^-1;
  P = (eye(2) - (K * H)) * P;
  x = x + K * ([phi(i); phi_dot(i)] - (H * F * x));
  
  % Grab the estimated state
  phi_output(i) = x(1);
end

figure(1);
plot(phi);
hold on;
plot(phi_output);
legend('phi', 'phi filtered');
