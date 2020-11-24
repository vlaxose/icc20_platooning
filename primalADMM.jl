# Implementation of Algoritm 1 from R. Rostami et. al, "ADMM-Based
# Distributed Model Predictive Control: Primal and Dual Approaches", IEEE
# CDC 2017

using Convex, SCS

function primalADMM(Aeq_i, Beq_i, Q, R, ysum, caligN, caligM, N, M, i)

    T = 10
    # calcuate the dimensions for the i-th subsystem
    qi = 0;
    yi = [];
    for j in caligN[i]
        n[j], ~ = size(Q[:,:,j])
        m[j], ~ = size(R[:,:,j])
        K = N*(n[j]+m[j])
        qi = qi + K
        yi = [yi; zeros(K,1)]
    end

    # initialize
    gammai = zeros(qi, 1)
    ysumi = zeros(qi, 1)
    rho = 0.1

    # create the local weighting matrices
    RQ[i] = [R[:,:,i] zeros(m[i], n[i]) ; zeros(n[i], m[i]) Q[:,:,i]
    H[1] = RQ[1]
    
    for i in 2:N
        H[i] = [H_i, zeros(m[i], n[i]) ; zeros(n[i], m[i]), RQ_i]
    end

    # local iterations at the i-th subsystem
    for t in 1:T
      # line 4
      yi_admm = Variable(qi)
      problem = minimize(quadform(yi_admm[1:N*(n[i]+m[i])], H[i])+ gammai'*(yi_admm-ysumi) + rho/2*norm(yi_admm-ysumi), [Aeq_i[:,:,i,j]*yi_admm==Beq_i])
      solve!(problem, SCS.Optimizer)
      yi(i,j)=yi_admm.value

      # line 5
      # send y_j to all j \in outConnections

      # line 6
      for j in 1:length(caligN_i)
          indx = 1+(j-1)*N*(m_i+n_i):j*N*(m_i+n_i)
          ysumj = zeros(qi, 1)
          for i in caligM_i
            ysumi[indx] = ysumi[indx] + ysumj[indx]
          end
      end

      # line 7
      # send ysum_j to all j \in inConnections

      # line 8
      for j \in caligN[i]
          y_i = 0
          for j \in caligM[i]

              y_i = y_i + yi[indx]
          end
      end
      # line 9
      gammai = gammai + rho*(yi - ysumi)
    end


end
