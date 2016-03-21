function [ output_args ] = plotTrajectory(dir_data, color, type)
%PLOT_TRAJECTORY Summary of this function goes here
%   Detailed explanation goes here

dir = sprintf('%s/%s/%s', dir_data.root, dir_data.package_name);
dir = sprintf('%s/%s', dir, dir_data.data_directory_name);
dir = sprintf('%s/%s', dir, dir_data.doc_name);

fid = fopen(dir);
X_text = textscan(fid, '%s', 12, 'delimiter', sprintf('\t'));
X = textscan(fid, '%f %f %f %f %f %f %f %f %f %f %f %f');
fclose(fid);

x = X{1,1}; xd = X{1,5};
y = X{1,2}; yd = X{1,6};
z = X{1,3}; zd = X{1,7};
p = X{1,4}; pp = X{1,8};

if strcmp(type, '3D') == 1
    plot3(x, y, z, color, 'LineWidth', 2)
elseif strcmp(type, '2D') == 1
    subplot(2,2,1); plot(x, color, 'LineWidth', 2);
    hold on
    ylabel('x(t)')
    grid

    subplot(2,2,2); plot(y, color, 'LineWidth', 2);
    hold on
    ylabel('y(t)')
    grid

    subplot(2,2,3); plot(z, color, 'LineWidth', 2);
    hold on
    xlabel('time [msec.]')
    grid
    ylabel('z(t)')

    subplot(2,2,4); plot(p, color, 'LineWidth', 2);
    hold on
    xlabel('time [msec.]')
    ylabel('pitch(t)')
    grid
end

hold on
end