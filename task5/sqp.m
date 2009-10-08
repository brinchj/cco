function [reserr angles] = sqp( goal, t, angles )

%
% min  (g-f)'(g-f)
% s.t. theta >= I, theta <= u
%


%
% HINT:
% Theta* = min h(theta)
%     s.t. theta >= l
%          theta <= u
% Where l <= 0 <= u
% and   h(theta) = 0.5 (g - f(theta))^T (g - f(theta))
%


%
% Local SQP
% for k = 0, 1, 2, ...:
%   find f_k, \delta f_k, c_k and A_k
%   W_k := W(x_k, l_k)
%
%   find p_k, u_k by solving 18.12
%   x_{k+1} := x_k + p_k
%   l_{k+1} := u_k
%
%   if convergence test satified
%      return (x_{k+1}, l_{k+1})
%   fi
% rof
%

end