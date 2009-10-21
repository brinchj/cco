function [ lambda count ] = solve_lcp(A, b, lambda)
% SOLVE_LCP - Solve the LCP problem
% input:
%    A    - A square symmetric positive (semi) definite matrix.
%    b    - The ``right-hand-side'' vector.
%  lambda - Initial guess of the solution.
%
% output:
%
%  lambda -  The resulting solution computed by the function.
%
% Copyright 2009, Kenny Erleben, DIKU:


    % Pseudo Code:
    % While not converged
    %  lambda = max(0, -inv( L+D)*(U*lambda + b)
    %  residual = min(A*lambda+b,lambda)
    %  error = residual'*residual
    % End

    fprintf(1, '>> SOLVER STARTED\n');

    % Expand Previous Lambda Vector To Fit A
    if (size(lambda, 1) < size(A, 1))
        lambda = [ lambda; zeros(size(A, 1) - size(lambda,1), 1) ];
    elseif (size(lambda, 1) > size(A, 1))
        lambda = lambda(1:size(A,1));
    end

    % Compute M and N
    M = tril(A);
    N = M - A;
    assert (all(all(A == M-N)));

    % Compute Matrix Inverse Using LU Decomposition (more precise)
    [L1 U] = lu(M);
    Mi = inv(M); %inv(L1) * inv(U);

    % Compute Initial Error
    residual = min(A*lambda + b, lambda);
    error = residual' * residual;
    fprintf(1, 'Error: %d\n', error);
    res = [error];

    count = 1;
    while error > 0.00001 && count < 20000
        newLambda   = max(0, Mi*(N*lambda - b));

        lambda = newLambda;
        residual = min(A*lambda + b, lambda);
        newError = residual' * residual;
        if error <= newError
            break
        end
        error = newError;

        fprintf(1, 'Error: %d -> %d\n', count, error);
        count = count + 1;
        res   = [res error];
    end

    %if count > 100
    %     name = strcat('plots_conv/s', sprintf('%04d', count), '.eps');
    %     fig = figure(2);
    %     semilogy(res);
    %     print('-deps', name);
    %     close(fig);
    % end

end
