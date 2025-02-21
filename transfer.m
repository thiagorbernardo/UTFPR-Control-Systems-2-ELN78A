A = csvread("train_data.csv");
time = A(:,1);
goal = A(:,2);
distance = A(:,3);

figure;
subplot(2,1,1);
plot(time, goal, 'r', 'LineWidth', 1.5);
title('Input Signal');
xlabel('Time (s)');
ylabel('Goal (cm)');

subplot(2,1,2);
plot(time, distance, 'b', 'LineWidth', 1.5);
title('Output Signal');
xlabel('Time (s)');
ylabel('Distance (cm)');
grid on;



B = csvread("test_data.csv");
time_test = B(:,1);
goal_test = B(:,2);
distance_test = B(:,3);








