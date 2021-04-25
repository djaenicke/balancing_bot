% Filtered output
phi_output = zeros(size(phi, 1), 1);
alpha = 0.98;
dt = 0.005;
gyro_bias = 2.5;
phi_ouptut(1) = phi(1);

for i=2:size(phi, 1)
  gyro_angle = (phi_dot(i) - gyro_bias) * dt;
  phi_output(i) = alpha*(phi_output(i-1) + gyro_angle) + ((1 - alpha) * phi(i));
end

figure(4);
plot(phi);
hold on;
plot(phi_output);
legend('phi', 'phi filtered');
