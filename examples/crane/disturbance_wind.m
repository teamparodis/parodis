function d = disturbance_wind(~, crane, ~, ~)
x_crane = crane.history.x(:, end);

v_cat = x_crane(2);
angle_rope = x_crane(3);
angle_velo_rope = x_crane(4);

rho_air = 1.2;
v_wind = 10;
% container face area
A_c = 4;
% rope length
L = 10;

F_wind = 1/2 * rho_air * (v_wind + v_cat + L*angle_velo_rope * cos(angle_rope))^2 * A_c * cos(angle_rope);
% wrap in cell
d = {F_wind};