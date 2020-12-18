clc
clear
numOfVehicles = 5;
safeDistance = 10;

dt = 0.1;
[waypoints,totalTime,leadingVehicleSpeed] = createWaypoints('const', dt);
[position_mpc, velocity_mpc] = controlModule(waypoints, dt, totalTime, numOfVehicles, safeDistance);

%% plot the position tracking
t = 1:dt:totalTime;
figure;
subplot(2, 1, 1)
for k=1:numOfVehicles
  plot(t, position_mpc(k,:)); hold on;
  lndstr{k}=char(['Car ', num2str(k)]);
end
title('MPC controller');
plot(t, waypoints, 'r--')
xlabel('time (s)')
ylabel('Platoon vehicles positions (m)')
lndstr{numOfVehicles+1} = char('Leading vehicle');

legend(lndstr)
grid on;
subplot(2, 1, 2)
lndstr={};
for k=1:numOfVehicles
  plot(t, velocity_mpc(k,:)); hold on;
  lndstr{k}=char(['Car ', num2str(k)]);
end
plot(t, leadingVehicleSpeed, 'r--')
lndstr{numOfVehicles+1} = char('Leading vehicle');
xlabel('time (s)')
ylabel('speed (m/s)')
grid on;
legend(lndstr)

savefig(['./results/pathplanning_mpc_const.fig'])
saveas(gcf,['./results/pathplanning_mpc_const.eps'],'epsc')
