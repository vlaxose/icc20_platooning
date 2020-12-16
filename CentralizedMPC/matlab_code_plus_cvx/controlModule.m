function [position,velocity] = controlModule(waypoints, dt, totalTime, numOfVehicles, safeDistance)


t = 1:dt:totalTime; % time instances where the controller gives output

% mass-spring equivalent parameters
m = 5;  % [kg]
c1 = 2; % [N/m]
c2 = 1;

if(numOfVehicles>1)
  E = ones(numOfVehicles);
else
  E = 0;
end
Lg = eye(numOfVehicles) - diag(diag(E,1),1);

position = zeros(numOfVehicles, length(t));
for k=1:numOfVehicles
  position(k, 1) = waypoints(1,1)-(k-1)*safeDistance;
end
velocity = zeros(numOfVehicles, length(t));

%% controller paramters and initialization

As = [0 1 ; -c1/m -c2/m];
Bs = [0 ; 1/m];
Cs = [1 0];
A = kron(Lg, As);
B = kron(Lg, Bs);        
C = kron(Lg, Cs);       

% MPC parameters
Np = 10;
Nc = 5; %Nc <= Np
o = zeros(numOfVehicles, size(A,1));
Ampc = [A o' ; C*A eye(numOfVehicles)];
Bmpc = [B ; C*B];
Cmpc = [o eye(numOfVehicles)];
X  = zeros(size(Ampc, 1), length(t));
DX  = zeros(size(Ampc, 1), length(t));

F = []; Phi = [];
for i=1:Np
  F = [F ; Cmpc*Ampc^i];
  phi = [];
  for j=1:Nc
      if(j<=i)
        phi = [phi  Cmpc*Ampc^(i-j)*Bmpc];
      else
        phi = [phi zeros(numOfVehicles, size(B, 2))];
      end
  end
  Phi = [Phi ; phi];
end
        
    %% Runtime
for t = numOfVehicles+1:length(t)
        t

     for k=1:numOfVehicles
%                   rs(k, 1) = waypoints(t-(k-1)) - waypoints(t-(k-1)-1);
        rs(k, 1) = waypoints(t-(k-1)) - position(k, t-1);

     end
     Rs = kron(ones(Np,1), rs);

     Psi = (Phi'*Phi + 1e-3*eye(Nc*numOfVehicles));
     beta = (Phi'*(Rs - F*DX(:, t-1)));
     
%      DU_cvx = pinv(Psi)*beta;

     cvx_begin quiet
          variable DU_cvx(Nc*numOfVehicles)
          minimize norm(beta-Psi*DU_cvx)
          subject to
                                        
          DXX = Ampc*DX(:, t-1) + Bmpc*DU_cvx(1:numOfVehicles);
          for k=1:numOfVehicles
            abs(X(2*k, t) + DXX(2*k))/dt <= 10;
            if(k>1)
              pos_k = X(k, t) + DXX(k);
              pos_k_1 = X(k-1, t) + DXX(k-1);
              abs(pos_k-pos_k_1) <= 10;
            end
          end
     cvx_end

     DX(:, t) = Ampc*DX(:, t-1) + Bmpc*DU_cvx(1:numOfVehicles);
     X(:, t+1) = X(:, t) + DX(:, t);

     position(:, t) = position(:, t-1)  + X([2:2:2*numOfVehicles], t+1);
     velocity(:, t) = X([2:2:2*numOfVehicles], t)/dt;
end
