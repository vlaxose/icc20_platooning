function [position,velocity] = controlModule(waypoints, dt, totalTime, numOfVehicles, safeDistance)


t = 1:dt:totalTime; % time instances where the controller gives output

if(numOfVehicles>1)
  E = ones(numOfVehicles);
else
  E = 0;
end
Lg = eye(numOfVehicles) - diag(diag(E,1),1) - diag(diag(E,-1),-1);

position = zeros(numOfVehicles, length(t));
velocity = zeros(numOfVehicles, length(t));
X  = zeros(3*numOfVehicles, length(t));
DX  = zeros(3*numOfVehicles, length(t));
        
%% Runtime
for t = numOfVehicles+1:length(t)
  t
  for k=1:numOfVehicles

      [position(k, t), velocity(k, t), X, DX] = dist_controlModule(Lg, position, waypoints, safeDistance, X, DX, k, t, dt);
      
  end  
end
