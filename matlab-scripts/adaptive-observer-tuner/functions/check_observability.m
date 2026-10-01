%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File:       check_observability.m                                                   
% Author:     Giri Mugundan Kumar                                             
% Date:       November 1, 2025                                                  
% For info:   Andrea L'Afflitto                                               
%             a.lafflitto@vt.edu                                              
%                                                                             
% Description: Function checks and prints the controllability of a system.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function check_observability(A, C)
    % CHECK_OBSERVABILITY Verifies observability and prints result.
    %   check_observability(A, C) computes the observability matrix,
    %   its rank, and prints whether the system (A, C) is observable.

    n = size(A, 1);
    Ob = obsv(A, C);
    r = rank(Ob);

    fprintf('Observability Matrix Rank: %d (Should be %d for observable system)\n', r, n);
    if r == n
        disp('System is observable.');
    else
        error('System is NOT observable.');
    end
end
