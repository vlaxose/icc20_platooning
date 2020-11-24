clc
clear
numOfVehicles = 3;
safeDistance = 10;

dt = 0.5;
[waypoints,totalTime,leadingVehicleSpeed, t] = createWaypoints(dt);
% [position_pid, velocity_pid] = myPlatoonBlock('PID', waypoints, dt, totalTime, numOfVehicles, safeDistance);
[position_mpc, velocity_mpc] = myPlatoonBlock('MPC', waypoints, dt, totalTime, numOfVehicles, safeDistance);
% [position_distmpc, velocity_distmpc] = myPlatoonBlock('distMPC', waypoints, dt, totalTime, numOfVehicles, safeDistance);
[position_distmpc2, velocity_distmpc2] = myPlatoonBlock('distMPC2', waypoints, dt, totalTime, numOfVehicles, safeDistance);

% 
% 
% plot the position tracking
% figure;
% subplot(2, 1, 1)
% for k=1:numOfVehicles
%   plot(t, position_pid(k,:)); hold on;
%   lndstr{k}=char(['Car ', num2str(k)]);
% end
% title('PID controller');
% plot(t, waypoints+safeDistance, 'r--')
% xlabel('time (s)')
% ylabel('Platoon vehicles positions (m)')
% lndstr{numOfVehicles+1} = char('Leading vehicle');
% legend(lndstr)
% 
% grid on;
% subplot(2, 1, 2)
% lndstr={};
% for k=1:numOfVehicles
%   plot(t, velocity_pid(k,:)); hold on;
%   lndstr{k}=char(['Car ', num2str(k)]);
% end
% plot(t, repmat(leadingVehicleSpeed, 1, length(t)), 'r--')
% lndstr{numOfVehicles+1} = char('Leading vehicle');
% xlabel('time (s)')
% ylabel('speed (m/s)')
% grid on;
% legend(lndstr)
% savefig(['./results/pid.fig'])
% saveas(gcf,['./results/pid.eps'],'epsc')

% plot the position tracking
figure;
subplot(2, 1, 1)
for k=1:numOfVehicles
  plot(t, position_mpc(k,:)-(k-1)*safeDistance); hold on;
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
plot(t, repmat(leadingVehicleSpeed, 1, length(t)), 'r--')
lndstr{numOfVehicles+1} = char('Leading vehicle');
xlabel('time (s)')
ylabel('speed (m/s)')
grid on;
legend(lndstr)

% % plot the position tracking
% figure;
% subplot(2, 1, 1)
% for k=1:numOfVehicles
%   plot(t, position_distmpc(k,:)-(k-1)*safeDistance); hold on;
%   lndstr{k}=char(['Car ', num2str(k)]);
% end
% title('Distributed MPC controller');
% plot(t, waypoints, 'r--')
% xlabel('time (s)')
% ylabel('Platoon vehicles positions (m)')
% lndstr{numOfVehicles+1} = char('Leading vehicle');
% 
% legend(lndstr)
% grid on;
% subplot(2, 1, 2)
% lndstr={};
% for k=1:numOfVehicles
%   plot(t, velocity_distmpc(k,:)); hold on;
%   lndstr{k}=char(['Car ', num2str(k)]);
% end
% plot(t, repmat(leadingVehicleSpeed, 1, length(t)), 'r--')
% lndstr{numOfVehicles+1} = char('Leading vehicle');
% xlabel('time (s)')
% ylabel('speed (m/s)')
% grid on;
% legend(lndstr)
% 

% plot the position tracking
% figure;
% subplot(2, 1, 1)
% for k=1:numOfVehicles
%   plot(t, position_distmpc2(k,:)-(k-1)*safeDistance); hold on;
%   lndstr{k}=char(['Car ', num2str(k)]);
% end
% title('Distributed MPC controller 2');
% plot(t, waypoints, 'r--')
% xlabel('time (s)')
% ylabel('Platoon vehicles positions (m)')
% lndstr{numOfVehicles+1} = char('Leading vehicle');
% 
% legend(lndstr)
% grid on;
% subplot(2, 1, 2)
% lndstr={};
% for k=1:numOfVehicles
%   plot(t, velocity_distmpc2(k,:)); hold on;
%   lndstr{k}=char(['Car ', num2str(k)]);
% end
% plot(t, repmat(leadingVehicleSpeed, 1, length(t)), 'r--')
% lndstr{numOfVehicles+1} = char('Leading vehicle');
% xlabel('time (s)')
% ylabel('speed (m/s)')
% grid on;
% legend(lndstr)
