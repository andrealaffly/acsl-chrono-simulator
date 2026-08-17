%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025 Giri M. Kumar, Andrea L'Afflitto. All rights reserved.                                                        
%                                                                             
% Redistribution and use in source and binary forms, with or without          
% modification, are permitted provided that the following conditions 
% are met: 
%                                                                             
% 1. Redistributions of source code must retain the above copyright notice,   
%    this list of conditions and the following disclaimer.                    
%                                                                             
% 2. Redistributions in binary form must reproduce the above copyright        
%    notice, this list of conditions and the following disclaimer in the      
%    documentation and/or other materials provided with the distribution.     
%                                                                             
% 3. Neither the name of the copyright holder nor the names of its            
%    contributors may be used to endorse or promote products derived from     
%    this software without specific prior written permission.                 
%                                                                             
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
% LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
% PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER 
% OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
% EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
% PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
% PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
% LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
% NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                                 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File:       adaptive_rotational_differentiator_parameter_generator
% Author:     Giri Mugundan Kumar                                             
% Date:       November 1, 2025 (first draft)                                                 
%             January 20, 2026 (added 2L params)
% For info:   Andrea L'Afflitto                                               
%             a.lafflitto@vt.edu                                              
%                                                                             
% Description: Main function to compute the params for the adaptive 
%              differentiator for the simulator and flight.
% 
% Github: https://github.com/girimugundankumar/acsl-physics-sim.git
%
% Note: AI was used to generate code for 'JSON Export Script' - Giri.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 3 c's
close all; clear; clc;

% Add path to the functions folder
addpath("functions/");

%% =======================================================================
%%%%%%%%%%%%%%%%%% ADAPTIVE OBSERVER TUNABLE PARAMETERS %%%%%%%%%%%%%%%%%%

% GAINS FOR THE A_ref_y MATRIX
K_P_ref_y = blkdiag(2.5, ...
                    2.5, ...
                    2.5);

K_D_ref_y = blkdiag(7.86, ...
                    10.08, ...
                    2.0);

% GAINS FOR THE A_tran_y MATRIX
K_P_tran_y = [2.0,  0.0,  0.0;
              0.0,  2.0,  0.0;
              0.0,  0.0,  2.0];
K_D_tran_y = [ 7.86,   0.0,   0.0;
                0.0, 10.08,   0.0;
                0.0,   0.0,  5.48];

% BOOLEANS FOR HOW YOU WANT TO CONTRUCT THE PLANT MATRICES
build_transient_A_from_reference_A = true;
build_reference_A_from_transient_A = false;

% OBSERVER ADAPTIVE RATES
param.Gamma_y = blkdiag(0.16, ...
                        0.36, ...
                        0.145);

param.Gamma_Theta_y = blkdiag(0e-3, ...
                              0e-3, ...
                              0e-3); 

param.Gamma_g_y = blkdiag(0.2, ...
                          0.2, ...
                          0.2);

% OBSERVER GAINS PROJECTION OPERTAOR PARAMETERS
param.projection_x_max_K_hat_y_differentiator = 200;
param.projection_epsilon_K_hat_y_differentiator = 5;

param.projection_x_max_Theta_hat_differentiator = 0.5;
param.projection_epsilon_Theta_hat_differentiator = 0.01;

% OBSERVER GAINS DEADZONE SWITCH TOLERANCE
param.projection_x_max_K_hat_g_y_differentiator = 200;
param.projection_epsilon_K_hat_g_y_differentiator = 5;

%% -----------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%% DO NOT MODIFY BEYOND THIS COMMENT %%%%%%%%%%%%%%%%%%%

% Physical properties and parameters
param.g = 9.80665;                             % gravity const [m/s^2]
param.mass = 1.688177479629479948997072824568; % mass of drone [Kg]
param.rho = 1.225;                             % air density [kg/m^3]

% Linear system matrices for translational plant
% Build A = [0_{3x3}, I_3; 0_{3x3}, 0_{3x3}]
param.A = [zeros(3),   eye(3);
           zeros(3), zeros(3)];

% Build B = [0_{3x3}; I_3]
param.B = [zeros(3,3);
               eye(3)];

% Build C = [0_{3x3}, I_3]
% param.C = [zeros(3,3), eye(3)];
param.C = [eye(3), eye(3)];

% System parameters for adaptive differentiator
% Build the Lambda
param.Lambda = 1;
param.lambda_bar = norm(param.Lambda);

% Specify the Theta (parametric uncertainties)
param.Theta = eye(3);
param.theta_bar = norm(param.Theta);

% Reference plant dynamics (closed-loop)
% Build A_ref = [0_{3x3}, I_3; -K_P_ref_y; -K_D_ref_y]
param.A_ref_y = [zeros(3), eye(3); -K_P_ref_y, -K_D_ref_y];

% Build A_tran = [0_{3x3}, I_3; -K_P_tran_y; -K_D_tran_y]
param.A_tran_y = [zeros(3), eye(3); -K_P_tran_y, -K_D_tran_y];

% ---------------------------------------------------------- REF Matrices
% Sanity Check 
if(build_transient_A_from_reference_A && build_reference_A_from_transient_A)
    error("You can only contruct A_ref or A_tran based on the other." + ...
          " Check the boolean conditions")
end

% Reference plant dynamics (closed-loop) build A_ref_y from A_tran_y
if (build_reference_A_from_transient_A && ~build_transient_A_from_reference_A)
    for ii = 0:0.0001:1
        [param.A_ref_y,param.found] = generate_A_ref_y(param.A_tran_y, ii);
        if (param.found)
            break;
        end
    end
end

% Reference plant dynamics (closed-loop) build A_tran_y from A_ref_y
if (build_transient_A_from_reference_A && ~build_reference_A_from_transient_A)
    for ii = 1:1:200000
        [param.A_tran_y, param.found] = generate_A_tran_y(param.A_ref_y, ii);
        if (param.found)
            break;
        end
    end
end

% ----------------------------------------------------------

% Obtain the A_ref_y matrix based on the A_tran_y matrix satisfying
% contraint. Eqn 7
check_eigen_condition(param.A_tran_y, param.A_ref_y)

% Check the controllability 
check_controllability(param.A_tran_y, param.B)
check_observability(param.A_tran_y, param.C)

% Lyapunov stability solution
% Build the Lyapunov equation's solution
param.Q_tran = 1*eye(6);                   % User-defined and POSITIVE-definite
param.P = lyap(param.A_tran_y', param.Q_tran);

% Build the optimal gains
param.K_ye = pinv(param.B * param.Lambda) * (param.A_ref_y - param.A) * pinv(param.C);
param.Theta_e_differentiator = zeros(3,3);
param.K_gye = pinv(param.B * param.Lambda) * (param.A_tran_y - param.A_ref_y) * pinv(param.C);

%% -----------------------------------------------------------------------
% Start finding the other parameters
disp('=== Starting 2-level tuning: (eps,L) with LMI; (Gamma) with GA ===');

% Find the n and m of the system
param.n = size(param.A_tran_y,1);
param.m = size(param.C, 1);
param.N = size(param.Theta, 2);

% Create a grid of eps values - This is so that we fix the epsilon and
% solve for the other parameters
eps_grid = [1e-8 1e-7 1e-6 1e-5 1e-4 1e-3 1e-2 ...
            0.02 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 ...
            1.0 1.1 1.2 1.3 1.4 1.5 1.6 1.7 1.8 1.9 ...
            2.0 2.1 2.2 2.3 2.4 2.5 2.6 2.7 2.8 2.9];

best_gap = -Inf;   % Define the best gap
have_best = false; % Boolean for when we have the best gap

for eps0 = eps_grid
cvx_begin sdp quiet
    cvx_precision high

    % Decision vars
    variable P(param.n, param.n) symmetric
    variable Y(param.n, param.m) 
    variable Z(param.n, param.n) symmetric % W'W
    variable lb_lambda_min_P nonnegative   % lower bound on λ_min(P)
    variable lb_lambda_min_Z nonnegative   % lower bound on λ_min(Z)
    variable ub_sq_norm_L nonnegative      % γ^2  (upper bound on ||L||_2^2) 

    % PSD lower bounds
    P - lb_lambda_min_P*eye(param.n) >= 0;
    Z - lb_lambda_min_Z*eye(param.n) >= 0;

    % Numerical floors
    P >= 1e-6*eye(param.n);
    Z >= 1e-8*eye(param.n);

    % Eqn (14a)
    -( param.A_tran_y'*P + P*param.A_tran_y - param.C'*Y' - Y*param.C + eps0*P + Z ) >= 0;

    % Schur complement:  [ P  Y; Y'  ub_sq_norm_L*I ] >= 0  ⇒  ||L||_2 ≤ γ = sqrt(ub_sq_norm_L)
    % Use this to bound L
    [ P, Y; Y', ub_sq_norm_L*eye(param.m) ] >= 0;

    % Obejective: maximize LHS terms (lb_lambda_min_P, lb_lambda_min_Z),
    %             minimize RHS (ub_sq_norm_L), regularize P
    w_lb_lambda_min_P   = 1;
    w_lb_lambda_min_Z  = 1;
    w_ub_sq_norm_L  = 0.2; 
    w_trace_P = 1e-3;

    minimize( - (w_lb_lambda_min_P*lb_lambda_min_P + w_lb_lambda_min_Z*lb_lambda_min_Z) + w_ub_sq_norm_L*ub_sq_norm_L + w_trace_P*trace(P))
cvx_end
    
% Recover and evaluate Eqn (15) with γ
L_cand  = P \ Y;
gamma   = sqrt(ub_sq_norm_L);
lhs_815 = eps0*lb_lambda_min_P + lb_lambda_min_Z;
rhs_815 = 2 * param.theta_bar * param.lambda_bar * gamma * norm(param.C,2);
gap_815 = lhs_815 - rhs_815;

fprintf('eps=%5.3f : t=%8.3e mu=%8.3e γ=%6.3f | LHS=%7.4f RHS=%7.4f GAP=%7.4f\n',...
        eps0, lb_lambda_min_P, lb_lambda_min_Z, gamma, lhs_815, rhs_815, gap_815);

% since we cannot make the Eqn (15) strictly verified, choose the best gap    
    if gap_815 > best_gap
        best_gap  = gap_815;
        BEST.eps  = eps0;
        BEST.L    = L_cand;
        BEST.P_tran_y   = P;
        BEST.lb_lambda_min_Z   = lb_lambda_min_Z;
        BEST.gamma = gamma;
        BEST.lb_lambda_min_P = lb_lambda_min_P;
        have_best = true;
    end

end

if ~have_best
    error('No feasible (eps,L) found.');
end

L = BEST.L;   % final differentiator gain for GA
param.L = L;  % Assign it to the params
fprintf('Selected eps=%.6g with ||L||=%.4g (gap=%.4g)\n', BEST.eps, norm(L,2), best_gap);

%% === OBSERVER JSON Export Script (with projection parameters) ===
% Requires helper functions:
%   clean_val.m
%   fmtnum.m
%   write_matrix.m
% and assumes variables exist in `param` struct and projection scalars exist in workspace.

outfile = 'differentiator_config.json';

% --- Clean matrices in param (in-place) ---
param.C = arrayfun(@(x) clean_val(x), param.C);
param.B = arrayfun(@(x) clean_val(x), param.B);
param.L = arrayfun(@(x) clean_val(x), param.L);
param.A_ref_y = arrayfun(@(x) clean_val(x), param.A_ref_y);
param.A_tran_y = arrayfun(@(x) clean_val(x), param.A_tran_y);
param.Gamma_y = arrayfun(@(x) clean_val(x), param.Gamma_y);
param.Gamma_Theta_y = arrayfun(@(x) clean_val(x), param.Gamma_Theta_y);
param.Gamma_g_y = arrayfun(@(x) clean_val(x), param.Gamma_g_y);
param.K_ye = arrayfun(@(x) clean_val(x), param.K_ye);
param.Theta_e_differentiator = arrayfun(@(x) clean_val(x), param.Theta_e_differentiator);
param.K_gye = arrayfun(@(x) clean_val(x), param.K_gye);

% --- Open JSON file ---
fid = fopen(outfile, 'w');
if fid == -1
    error('Could not open %s for writing', outfile);
end

% ---- Write JSON manually for perfect control ----
try
    fprintf(fid, '{\n');
    fprintf(fid, '  "DIFFERENTIATOR": {\n');

    % List of matrix entries
    entries = { ...
        'C_differentiator', param.C; ...
        'B_differentiator', param.B; ...
        'L_differentiator', param.L; ...
        'A_ref_y_differentiator', param.A_ref_y; ...
        'A_tran_y_differentiator', param.A_tran_y; ...
        'Gamma_y_differentiator', param.Gamma_y; ...
        'Gamma_Theta_differentiator', param.Gamma_Theta_y; ...
        'Gamma_g_y_differentiator', param.Gamma_g_y;
        'K_ye_differentiator', param.K_ye; ...
        'Theta_e_differentiator', param.Theta_e_differentiator; ...
        'K_gye_differentiator', param.K_gye ...
    };

    nEntries = size(entries, 1);
    for k = 1:nEntries
        name = entries{k, 1};
        M = entries{k, 2};

        fprintf(fid, '    "%s": {\n', name);
        fprintf(fid, '      "scaling_coef": 1,\n');
        fprintf(fid, '      "matrix": [\n');

        write_matrix(fid, M, '        ');

        fprintf(fid, '      ]\n'); % close matrix

        % Add comma if not the last matrix entry
        if k < nEntries
            fprintf(fid, '    },\n');
        else
            fprintf(fid, '    },\n'); % we'll add projection params after this
        end
    end

    % --- Projection Parameters ---
    % Clean and format values before writing
    fprintf(fid, '    "projection_x_max_K_hat_y_differentiator": %s,\n', ...
        fmtnum(param.projection_x_max_K_hat_y_differentiator));
    fprintf(fid, '    "projection_epsilon_K_hat_y_differentiator": %s,\n', ...
        fmtnum(param.projection_epsilon_K_hat_y_differentiator));

    fprintf(fid, '    "projection_x_max_Theta_hat_differentiator": %s,\n', ...
        fmtnum(param.projection_x_max_Theta_hat_differentiator));
    fprintf(fid, '    "projection_epsilon_Theta_hat_differentiator": %s,\n', ...
        fmtnum(param.projection_epsilon_Theta_hat_differentiator));

    fprintf(fid, '    "projection_x_max_K_hat_g_y_differentiator": %s,\n', ...
        fmtnum(param.projection_x_max_K_hat_g_y_differentiator));
    fprintf(fid, '    "projection_epsilon_K_hat_g_y_differentiator": %s,\n', ...
        fmtnum(param.projection_epsilon_K_hat_g_y_differentiator));

    fprintf(fid, '    "lambda_bar_differentiator": %s,\n', ...
        fmtnum(param.lambda_bar));
    fprintf(fid, '    "theta_bar_differentiator": %s\n', ...
        fmtnum(param.theta_bar));


    % --- Close differentiator block ---
    fprintf(fid, '  }\n');
    fprintf(fid, '}\n');

    fclose(fid);
    fprintf('✅ Wrote %s with matrices + projection parameters.\n', outfile);

catch ME
    fclose(fid);
    rethrow(ME);
end