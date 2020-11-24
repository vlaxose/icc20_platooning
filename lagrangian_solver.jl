function langragian_solver(H, ysum, A_i, Aeq, Beq_i, Bineq_i, qi, outConnections, gamma, rho)

  yi = Variable(qi)
  problem = minimize(quadform(yi,H) + gamma'*(yi-ysum) + rho/2*norm(yi-ysum), [Aeq*yi==Beq_i])

  solve!(problem, SCS.Optimizer)
  problem.status

  return yi

end
