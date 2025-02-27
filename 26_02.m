% Define A(z) and B(z)
A = [0 0.1294]; % Numerator coefficients (0.1294 z^-1)
B = [1 -1.185 0.1342 0.1803]; % Denominator coefficients

P = tf(A, B, 0.1);

% Compute R(z) = A(z) / (B(z) - A(z))
B_minus_A = B;
B_minus_A(2) = B_minus_A(2) - 0.1294; % Subtract A(z) from B(z)
R = tf(A, B_minus_A, 0.1); % Discrete-time transfer function with Ts = 0.1

% Define a range of K values to test
K_values = 0.1:0.05:0.4;

% Initialize a table to store step response metrics
metrics = table('Size', [length(K_values), 4], ...
                'VariableTypes', {'double', 'double', 'double', 'double'}, ...
                'VariableNames', {'K', 'RiseTime', 'SettlingTime', 'Overshoot'});

% Loop through each K value
for i = 1:length(K_values)
    K = K_values(i);
    
    % Compute the closed-loop transfer function
    closed_loop_tf = feedback(K * R, 1);
    
    % Get step response metrics
    step_info = stepinfo(closed_loop_tf);
    
    % Store metrics in the table
    metrics.K(i) = K;
    metrics.RiseTime(i) = step_info.RiseTime;
    metrics.SettlingTime(i) = step_info.SettlingTime;
    metrics.Overshoot(i) = step_info.Overshoot;
end

% Display the metrics table
disp(metrics);

% Plot step responses for different K values
figure;
hold on;
for K = K_values
    closed_loop_tf = feedback(K * R, 1);
    step(closed_loop_tf);
end
legend(cellstr(num2str(K_values', 'K = %.2f')));
title('Step Response for Different K Values');
xlabel('Time');
ylabel('Amplitude');
hold off;


K = 0.35;
T_num = K*R;
T_den = 1 + T_num;

Ts = 0.1;

T = tf(T_num, T_den, Ts);

C1 = pidtune(T, 'PID', Ts);

C = pidtune(P, 'PID', Ts);

% Compute closed-loop transfer function
closed_loop_pid = feedback(C1 * P, 1);

% Step response analysis
step(closed_loop_pid);
grid on;
title('Step Response with Discrete PID');
xlabel('Time (seconds)');
ylabel('Response');

