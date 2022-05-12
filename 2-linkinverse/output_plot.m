% Plot the value from the workspace 
close all;

q1_d = 2.5*t.^2 - 1.5*t.^3; 
q2_d = 3.5*t.^2 - 2.5*t.^3; 

figure(1);
subplot(211); 
plot(t, x1,'r', t, q1_d, 'b', 'LineWidth', 2);
xlabel('time(s)');ylabel('position tracking of link 1');
legend('q1','q1_d')
subplot(212);
plot(t, x2,'r', t, q2_d, 'b', 'LineWidth', 2);
xlabel('time(s)');ylabel('position tracking of link 2');
legend('q2','q2_d')

figure(2);
subplot(211);
plot(t,torque(:, 1), 'b', 'LineWidth', 2);
xlabel('time(s)');ylabel('torque1');
legend('torque1')
subplot(212);
plot(t,torque(:, 2), 'b', 'LineWidth', 2);
xlabel('time(s)');ylabel('torque');
legend('torque2')
