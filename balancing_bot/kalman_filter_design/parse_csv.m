data = load("-ascii", 'log.csv');
ts = data(:,1);
phi = data(:,2);
phi_dot = data(:,3);

figure(1);
plot(phi);
hold on;
plot(phi_dot);
legend('phi', 'phi dot');
