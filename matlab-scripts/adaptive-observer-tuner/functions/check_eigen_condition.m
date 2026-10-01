%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File:       checkEigenCondition.m                                      
% Author:     Giri Mugundan Kumar                                         
% Date:       November 1, 2025                                            
% For info:   Andrea L'Afflitto                                           
%             [a.lafflitto@vt.edu](mailto:a.lafflitto@vt.edu)             
%                                                                         
% Description: Function checks necessary condtion in Eqn (7)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = check_eigen_condition(A_tran_y, A_ref_y)
% checkEigenCondition Checks if
% max(Re(eig(A_tran_y))) < min(Re(eig(A_ref_y)))
%
% Usage:
%   satisfies = check_eigen_condition(A_tran_y, A_ref_y)
%
% Inputs:
%   A_tran_y - square matrix for the transition dynamics
%   A_ref_y  - square matrix for the reference dynamics
%
% Output:
%   satisfies - true if the condition holds, false otherwise

eig_tran = eig(A_tran_y);
eig_ref  = eig(A_ref_y);

max_re_tran = max(real(eig_tran));
min_re_ref  = min(real(eig_ref));

satisfies = max_re_tran < min_re_ref;

if satisfies
    fprintf('Condition satisfied:\n');
    fprintf('    max(Re(eig(A_tran_y))) = %.4f\n', max_re_tran);
    fprintf('    min(Re(eig(A_ref_y)))  = %.4f\n', min_re_ref);
    fprintf('    %.4f < %.4f\n', max_re_tran, min_re_ref);
else
    error(['Condition NOT satisfied:\n' ...
           '    max(Re(eig(A_tran_y))) = %.4f\n' ...
           '    min(Re(eig(A_ref_y)))  = %.4f\n' ...
           '    %.4f >= %.4f\n'], ...
           max_re_tran, min_re_ref, max_re_tran, min_re_ref);
end


end
