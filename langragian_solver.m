function [yi] = langragian_solver(H, ysum, A_i, Aeq, Beq_i, Bineq_i, qi, outConnections, gamma, rho)

  cvx_begin
    variable yi(qi, 1);

    minimize(1/2 * yi'*H*yi + gamma'*(yi-ysum) + rho/2*norm(yi-ysum))
%     subject to
%       q1=zeros(qi, 1);
%       for j=outConnections
%         q1 = q1 + Aeq(:,:,j)*yi;
%       end
%       q1 == Beq_i;

%       A_i*yi <= Bineq_i;

  cvx_end

end