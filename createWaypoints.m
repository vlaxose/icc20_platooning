function [waypoints,totalTime,leadingVehicleSpeed, t] = createWaypoints(dt)

    totalTime = 30;
    t = 1:dt:totalTime; % time instances where the controller gives output
    leadingVehicleSpeed = 10;
    
    waypoints = leadingVehicleSpeed*t; 
  
end