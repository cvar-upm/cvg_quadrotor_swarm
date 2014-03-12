function plot_3d_trajectory(trajectory)

plot3(-trajectory.pos(2, :), trajectory.pos(1, :), trajectory.pos(3, :));
xlabel('y'); ylabel('x'); zlabel('Altitude');
axis equal;
grid on;

end

