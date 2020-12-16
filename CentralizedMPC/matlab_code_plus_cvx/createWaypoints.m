function [waypoints,totalTime,leadingVehicleSpeed] = createWaypoints(type,dt)

  switch(type)
    
    case 'const'
     totalTime = 30;
     t = 1:dt:totalTime; % time instances where the controller gives output
     leadingVehicleSpeed = 10;
    
     waypoints = leadingVehicleSpeed*t; 
    
    case 'carla'
      load('data_from_Carla.mat');
      vid=88;
      indx = find(data_from_Carla(:, 2)==vid);
      waypoints_raw = data_from_Carla(indx, 3);

      totalTime = length(indx);
      waypoints_next = [waypoints_raw(2:end) ; waypoints_raw(end)];
      leadingVehicleSpeed_raw = (waypoints_next - waypoints_raw);
      leadingVehicleSpeed = reshape(repmat(leadingVehicleSpeed_raw(:)', round(1/dt), 1), round(1/dt)*totalTime, 1).';
      leadingVehicleSpeed = leadingVehicleSpeed(1:round(1/dt)*totalTime-1);
      
      waypoints = zeros(1, round(1/dt)*totalTime-1);
      waypoints(1) = waypoints_raw(1);
      for t=2:round(1/dt)*totalTime-1
       waypoints(t) = waypoints(t-1) + leadingVehicleSpeed(t)*dt;
      end
  end
end