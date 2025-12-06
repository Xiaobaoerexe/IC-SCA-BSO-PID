function plot_weight_comparison()
%% Plot Weight Comparison (Figure 5)
% Compares nonlinear weight omega1 with linear weight omega2

maxIter = 100;
a = 5;  % Weight optimization parameter

iterations = 1:maxIter;

% Nonlinear weight omega1 (Equation 22)
% omega = 0.5 * [(1 - log(1 + (e-1)*k/kmax)) + exp(-k/a)]
omega1 = zeros(1, maxIter);
for k = 1:maxIter
    omega1(k) = 0.5 * ((1 - log(1 + (exp(1)-1) * k/maxIter)) + exp(-k/a));
end

% Linear weight omega2 (traditional PSO style)
% Linear decrease from 0.9 to 0.4
omega2 = 0.9 - (0.9 - 0.4) * (iterations - 1) / (maxIter - 1);

% Plot comparison
figure('Name', 'Weight Comparison');
hold on;
plot(iterations, omega1, 'b-', 'LineWidth', 2, 'DisplayName', '\omega_1 (Nonlinear)');
plot(iterations, omega2, 'r--', 'LineWidth', 2, 'DisplayName', '\omega_2 (Linear)');
hold off;

xlabel('Iteration');
ylabel('Weight \omega');
title('Figure 5: Comparison of Weight \omega');
legend('Location', 'northeast');
grid on;
set(gca, 'FontSize', 11);

% Adjust figure size
set(gcf, 'Position', [100, 100, 600, 450]);

fprintf('Weight comparison plot generated.\n');
fprintf('Nonlinear weight (omega1):\n');
fprintf('  Initial value: %.4f\n', omega1(1));
fprintf('  Final value: %.4f\n', omega1(end));
fprintf('Linear weight (omega2):\n');
fprintf('  Initial value: %.4f\n', omega2(1));
fprintf('  Final value: %.4f\n', omega2(end));

end
