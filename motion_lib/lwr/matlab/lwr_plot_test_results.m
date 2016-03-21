function lwr_plot_test_results()

root = '/home/cmastalli/ROS_myThesis/motion_lib';

package_name = 'lwr';
% library_directory_name = 'library';
data_directory_name = 'test/data_test';

dir = sprintf('%s/%s/%s', root, package_name, data_directory_name);

test_x = load(sprintf('%s/x_test.txt', dir));
test_y = load(sprintf('%s/y_test.txt', dir));
test_xq = load(sprintf('%s/xq_test.txt', dir));
test_yp = load(sprintf('%s/yp_test.txt', dir));
% test_yp_copy = load(sprintf('%s/yp_test_copy.txt', dir));
basis_function_matrix = load(sprintf('%s/basis_function_matrix.txt', dir));

figure(1)
subplot(2, 1, 1)
hold on;
plot(test_x, test_y, '--k', 'LineWidth', 2.5);
plot(test_xq, test_yp, 'r', 'LineWidth', 2.5);
% plot(test_xq, test_yp_copy, '--g', 'LineWidth', 2);
title('LWR Model');
legend('Input', 'Prediction')
xlabel('x')
ylabel('y')
hold off;

subplot(2, 1, 2)
plot(basis_function_matrix(1:end,1:10));

