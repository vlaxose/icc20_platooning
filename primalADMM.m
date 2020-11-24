% Implementation of Algoritm 1 from R. Rostami et. al, "ADMM-Based
% Distributed Model Predictive Control: Primal and Dual Approaches", IEEE
% CDC 2017

% A : the system matrix of the global system
% B : the input matrix of the global system
% subSysSizes : vector with the size of each agent system matrix
% subConSizes : vecotr with the size of each agent input matrix
function primalADMM(A,B, Aeq, Bineq, numOfsub, subSysSizes,subConSizes,inConnections, outConnections, predictionLength)

  [n,m]=size(B);

  y = zeros(predictionLength*(n+m), numOfsub);

  for i=1:M

    qi = 0;
    for j=outConnections(:,i)
      qi = qi + predictionLength*(subSysSizes(j)+subConSizes(j));
    end
    gamma = zeros(qi, 1);
    ysum = zeros(qi, 1);

    for t=1:T
      % line 4



      % line 5

      % line 6
      ysum = 0;
      for j=inConnections(:,i)
        ysum = ysum + y
      end


      % line 7

      % line 8


      % line 9
      gamma = gamma + rho*(yi
    end
  end
end
