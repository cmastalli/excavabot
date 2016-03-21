function plot_motion_record_results()

root = '/home/cmastalli/ros_workspace/excavabot/motion_lib';

package_name = 'motion_record';
data_directory_name = 'data';

dir = sprintf('%s/%s/%s', root, package_name, data_directory_name);

% test_x = load(sprintf('%s/x_test.txt', dir));

fid = fopen(sprintf('%s/load_action.txt',dir));
X_text = textscan(fid, '%s', 12, 'delimiter', sprintf('\t'));
X = textscan(fid, '%f %f %f %f %f %f %f %f %f %f %f %f');
fclose(fid);

x = X{1,1}; xd = X{1,5};
y = X{1,2}; yd = X{1,6};
z = X{1,3}; zd = X{1,7};
p = X{1,4}; pp = X{1,8};

figure(1)
plot3(x, y, z, 'k', 'LineWidth', 3)
set(gca,'DataAspectRatio',[1 1 1])
set(gca,'CameraUpVector',[0 1 0])
set(gca,'CameraPosition',[20.3288 14.6317 -15.6895])
set(gca,'Projection','perspective')

title('Trajectory recorded')
xlabel(X_text{1,1}{1,1})
ylabel(X_text{1,1}{2,1})
zlabel(X_text{1,1}{3,1});
grid
hold on

num_data = size(X{:,1},1);

  for i = 1:5:num_data
      vel_mag = sqrt(xd(i)^2 + yd(i)^2 + zd(i)^2);
      hold on
      p0 = [x(i) y(i) z(i)];
      pf = p0 + [xd(i) yd(i) zd(i)]/(5*vel_mag);
      vectarrow(p0,pf)
  end

figure(2)
subplot(2,2,1); plot(x, 'k', 'LineWidth', 2);
ylabel('x(t)')

subplot(2,2,2); plot(y, 'k', 'LineWidth', 2);
ylabel('y(t)')

subplot(2,2,3); plot(z, 'k', 'LineWidth', 2);
xlabel('time [sec.]')
ylabel('z(t)')

subplot(2,2,4); plot(p, 'k', 'LineWidth', 2);
xlabel('time [sec.]')
ylabel('pitch(t)')

end
