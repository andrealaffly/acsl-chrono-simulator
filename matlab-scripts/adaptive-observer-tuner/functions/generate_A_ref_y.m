%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File:       generate_A_ref_y.m                                                   
% Author:     Giri Mugundan Kumar                                             
% Date:       November 24, 2025                                                  
% For info:   Andrea L'Afflitto                                               
%             a.lafflitto@vt.edu                                              
%                                                                             
% Description: Computes A_ref_y matix based on A_tran_y
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [A_ref_y, found] = generate_A_ref_y(A_tran_y, alpha)
% generate_A_ref_y creates A_ref_y given A_tran_y,
% setting all poles a margin alpha to the right of the fastest pole of A_tran_y.
%
% Inputs:
%   A_tran_y : System matrix (2n x 2n, second-order canonical structure)
%   alpha    : Margin (>0) to shift min real part of A_ref_y poles to the right
%
% Output:
%   A_ref_y  : Reference system matrix (2n x 2n canonical structure)

n = size(A_tran_y,1) / 2;
eig_tran = eig(A_tran_y);
max_real_tran = max(real(eig_tran));
desired_pole_real = max_real_tran + abs(alpha); % moves right (slower poles)

% Place all A_ref_y poles at this new value (real, critically damped)
K_P_ref_y = abs(desired_pole_real) * eye(n);
K_D_ref_y = 2 * sqrt(abs(desired_pole_real)) * eye(n);

A_ref_y = [zeros(n), eye(n); -K_P_ref_y, -K_D_ref_y];

% Show what we've constructed
eig_ref = eig(A_ref_y);
min_real_ref = min(real(eig_ref));
if min_real_ref > max_real_tran
    fprintf(['A_ref_y poles [%.4f] are all to the right of the fastest A_{tran,y} pole [%.4f] by margin alpha=%.2f.\n'], ...
        min_real_ref, max_real_tran, abs(alpha));
    found = true;
else
    warning('A_ref_y pole placement failed: slowest pole %.4f not right of fastest tran %.4f.\n', min_real_ref, max_real_tran);
    found = false;
end
end
