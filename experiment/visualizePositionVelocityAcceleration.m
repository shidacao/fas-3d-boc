function [] = visualizePositionVelocityAcceleration(pva, time)

persistent pos;
persistent vel;
persistent acc;

if time(1) == 0
    pos = [];
    vel = [];
    acc = [];
end

pos_now = pva(:, [1 2]);
vel_now = pva(:, [3 4]);
acc_now = pva(:, [5 6]);

pos = [pos; pos_now];
vel = [vel; vel_now];
acc = [acc; acc_now];

figure(1);
clf(1, 'reset');
hold on;
axis equal;
grid on; 
axis([-2, 2, -0.5, 2]);

is_only_display_some = false;
display_num = 3;
if is_only_display_some
    from = max(1, size(pos,1) - display_num + 1);
else
    from = 2;
end
to = size(pos,1);

plot(pos(:,1), pos(:,2), 'LineWidth', 2); 
for i = from : to
    quiver(pos(i,1), pos(i,2), vel(i,1), vel(i,2), 'red');
    quiver(pos(i,1), pos(i,2), acc(i,1), acc(i,2), 'black');
end

end % function
