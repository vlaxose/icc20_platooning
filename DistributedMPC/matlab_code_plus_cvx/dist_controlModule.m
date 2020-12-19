function [position_kt, velocity_kt, X, DX] = dist_controlModule(Lg, position, waypoints, safeDistance, X, DX, k, t, dt)

  numOfVehicles = length(Lg);
  
  % mass-spring equivalent parameters
  m = 5;  % [kg]
  c1 = 2; % [N/m]
  c2 = 1;

  As = [0 1 ; -c1/m -c2/m];
  Bs = [0 ; 1/m];
  Cs = [1 0];
  A = kron(Lg, As);
  B = kron(Lg, Bs);        
  C = kron(Lg, Cs);       

  % MPC parameters
  Np = 10;
  Nc = 5; %Nc <= Np
  o = zeros(numOfVehicles, 2*numOfVehicles);
  Ampc = [A o' ; C*A eye(numOfVehicles)];
  Bmpc = [B ; C*B];
  Cmpc = [o eye(numOfVehicles)];

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

  for kk=1:numOfVehicles
   if(k==kk)
    rs(kk, 1) = waypoints(t) - position(kk, t-1) - kk*safeDistance;
   else
    rs(kk, 1) = waypoints(t) - (0.999*position(kk, t-1)) -kk*safeDistance;
   end

  end
  Rs = kron(ones(Np,1), rs);

  Psi = (Phi'*Phi + 1e-3*eye(Nc*numOfVehicles));
  beta = (Phi'*(Rs - F*DX(:, t-1)));

  cvx_begin quiet
      variable DU_cvx(Nc*numOfVehicles)
      minimize norm(beta-Psi*DU_cvx)
      subject to

      DXX = Ampc*DX(:, t-1) + Bmpc*DU_cvx(1:numOfVehicles);
      for kk=1:numOfVehicles
        abs(X(2*kk, t) + DXX(2*kk))/dt <= 10;
      end
  cvx_end

  DX(:, t) = Ampc*DX(:, t-1) + Bmpc*DU_cvx(1:numOfVehicles);
  X(:, t+1) = X(:, t) + DX(:, t);

  position_kt = position(k, t-1)  + X(2*k, t+1);
  velocity_kt = X(2*k, t+1)/dt;
  
  end