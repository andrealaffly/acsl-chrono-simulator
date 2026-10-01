%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File:       check_controllability.m                                      
% Author:     Giri Mugundan Kumar                                         
% Date:       November 1, 2025                                            
% For info:   Andrea L'Afflitto                                           
%             [a.lafflitto@vt.edu](mailto:a.lafflitto@vt.edu)             
%                                                                         
% Description: Function checks and prints the controllability of a system.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function check_controllability(A, B)
    % CHECK_CONTROLLABILITY Verifies controllability and prints result.
    %   check_controllability(A, B) computes the controllability matrix,
    %   its rank, and prints whether the system (A, B) is controllable.

    n = size(A, 1);
    Ctrb = ctrb(A, B);
    r = rank(Ctrb);

    fprintf('Controllability Matrix Rank: %d (Should be %d for controllable system)\n', r, n);
    if r == n
        disp('System is controllable.');
    else
        error('System is NOT controllable.');
    end
end
