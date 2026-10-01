%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File:       generate_A_tran_y.m                                                   
% Author:     Giri Mugundan Kumar                                             
% Date:       November 24, 2025                                                  
% For info:   Andrea L'Afflitto                                               
%             a.lafflitto@vt.edu                                              
%                                                                             
% Description: Computes A_tran_y matix based on A_ref_y
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [A_tran_y, found] = generate_A_tran_y(A_ref_y, alpha)
% generate_A_tran_y constructs A_tran_y for a 2nd order block system,
% placing all poles at least alpha further left than the slowest in A_ref_y.
%
% Inputs:
%   A_ref_y : Reference system matrix, 2n x 2n, canonical 2nd order form
%   alpha   : Margin by which A_tran_y poles should be left of slowest A_ref_y pole (positive scalar)
%
% Output:
%   A_tran_y: System matrix with all poles at least alpha to the left of min pole in A_ref_y

n = size(A_ref_y, 1) / 2;
eig_ref = eig(A_ref_y);
min_real_ref = min(real(eig_ref));
desired_pole_real = min_real_ref - abs(alpha); % strictly further left

% Choose critically damped for all coordinates (real negative poles)
K_P_tran_y = abs(desired_pole_real) * eye(n);      % Diagonal: |desired_pole_real|
K_D_tran_y = 2 * sqrt(abs(desired_pole_real)) * eye(n);

A_tran_y = [zeros(n), eye(n); -K_P_tran_y, -K_D_tran_y];

% Optionally, check and print the eigenvalues
eig_tran = eig(A_tran_y);
max_re_tran = max(real(eig_tran));
if max_re_tran < min_real_ref
    fprintf('A_tran_y poles [%.4f] are all left of the slowest A_ref_y pole [%.4f] by margin alpha=%.2f.\n', ...
        max_re_tran, min_real_ref, abs(alpha));
        found = true;
else
    warning('A_tran_y pole placement failed: slowest pole %.4f not left of reference %.4f.\n', max_re_tran, min_real_ref);
    found = false;
end
end
