function plot_imitation_learning_results()

root = '/home/cmastalli/ROS_myThesis/motion_lib';

package_name = 'dynamic_movement_primitives';
data_directory_name = 'test/data_test';

dir = sprintf('%s/%s/%s', root, package_name, data_directory_name);

%% plot target function and target function learned
fid = fopen(sprintf('%s/truck_loading_dmp_ft.txt',dir));
F_text = textscan(fid, '%s', 8, 'delimiter', sprintf('\t'));
F = textscan(fid, '%f %f %f %f %f %f %f %f');
fclose(fid);

f1 = F{1,1}; f1_l = F{1,2};
f2 = F{1,3}; f2_l = F{1,4};
f3 = F{1,5}; f3_l = F{1,6};
f4 = F{1,8}; f4_l = F{1,8};

num_data = size(F{:,1},1);

for i = 1:num_data
    t(i) = i*0.01;
end   

figure(1)
subplot(2,2,1); plot(t, f1, 'k', 'LineWidth', 2);
hold on; plot(t, f1_l, '--r', 'LineWidth', 2);
ylabel('f_x(s)')
legend('demostrated', 'learned')

subplot(2,2,2); plot(t, f2, 'k', 'LineWidth', 2);
hold on; plot(t, f2_l, '--r', 'LineWidth', 2);
ylabel('f_y(s)')
legend('demostrated', 'learned')

subplot(2,2,3); plot(t, f3, 'k', 'LineWidth', 2);
hold on; plot(t, f3_l, '--r', 'LineWidth', 2);
ylabel('f_z(s)')
legend('demostrated', 'learned')

subplot(2,2,4); plot(t, f4, 'k', 'LineWidth', 2);
hold on; plot(t, f4_l, '--r', 'LineWidth', 2);
ylabel('f_{pitch}(s)')
legend('demostrated', 'learned')

% axis([0 1 -0.35 0.35])

%% plot trajectory demostrated and trajectory learned
fid = fopen(sprintf('%s/truck_loading_action.txt',dir));
traj_demo_text = textscan(fid, '%s', 12, 'delimiter', sprintf('\t'));
traj_demo = textscan(fid, '%f %f %f %f %f %f %f %f %f %f %f %f');
fclose(fid);

fid = fopen(sprintf('%s/truck_loading_dmp_traj_learned.txt',dir));
traj_learned_text = textscan(fid, '%s', 12, 'delimiter', sprintf('\t'));
traj_learned = textscan(fid, '%f %f %f %f %f %f %f %f %f %f %f %f');
fclose(fid);

x = traj_demo{1,1}; x_l = traj_learned{1,1};
y = traj_demo{1,2}; y_l = traj_learned{1,2};
z = traj_demo{1,3}; z_l = traj_learned{1,3};
p = traj_demo{1,4}; p_l = traj_learned{1,4};

figure(2)
subplot(2,2,1); plot(t, x, 'k', 'LineWidth', 2);
%hold on; plot(t, x_l, '--r', 'LineWidth', 2);
ylabel('x(t)')
%legend('demostrated', 'learned')

subplot(2,2,2); plot(t, y, 'k', 'LineWidth', 2);
%hold on; plot(t, y_l, '--r', 'LineWidth', 2);
ylabel('y(t)')
%legend('demostrated', 'learned')

subplot(2,2,3); plot(t, z, 'k', 'LineWidth', 2);
%hold on; plot(t, z_l, '--r', 'LineWidth', 2);
%xlabel('time [sec.]')
ylabel('z(t)')
%legend('demostrated', 'learned')

subplot(2,2,4); plot(t, p, 'k', 'LineWidth', 2);
%hold on; plot(t, p_l, '--r', 'LineWidth', 2);
%xlabel('time [sec.]')
ylabel('pitch(t)')
%legend('demostrated', 'learned')

end
