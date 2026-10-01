%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2026 Haoran Wang, Andrea L'Afflitto. All rights reserved.                                                        
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
% Author:     Haoran Wang                                             
% Date:       August 19, 2026
% For info:   Andrea L'Afflitto                                               
%             a.lafflitto@vt.edu                                              
%                                                                             
% Description: This script generate the basis center matrix for a given 3D
%               space. The basis centers are distributed on a 3D lattice.
%               Then with given basis function, the inverse grammian or
%               KK_inv matrix will be calculated. The resulting matrices
%               will be output into a JSON file which need to be manually
%               pasted into the drone parameter JSON file. The basis
%               function parameter need to be entered into the same file
%               manually. Calculated power function and condition number
%               are important for determining the quality of approximation. 
% 
% Github: https://github.com/haoran9vt/acsl-chrono-simulator
%
% Note: AI was used to generate code for 'JSON Export Script' - Haoran.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% House Keeping
clear all; close all; clc

%% Define Parameters
rangeX=[-2,2];                                  % X range
rangeY=[-2,2];                                  % Y range
rangeZ=[-2,2];                                  % Z range
N=3;                                            % Number of subdivisons of each dimension
m=3;                                            % Number of inputs
l=3;                                            % Hyperparameter value
knlfun = @(x,y) exp(-1/(2*l^2)*norm(x-y,2).^2)*eye(3);  % Kernel function (will update later)

%% Calculating Key Matrices
xcentergrid=linspace(rangeX(1),rangeX(2),N);                % x coordinates of centers
ycentergrid=linspace(rangeY(1),rangeY(2),N);                % y coordinates of centers
zcentergrid=linspace(rangeZ(1),rangeZ(2),N);                % y coordinates of centers

[Xcenter,Ycenter,Zcenter]=meshgrid(xcentergrid,ycentergrid,zcentergrid);        % creating meshgrid

basiscenter=[Xcenter(:),Ycenter(:),Zcenter(:)]';                       % Assigning basiscenters

% Calculating the Grammian (big K) matrix
k_xi_xi=zeros(length(basiscenter)*m);                         % Initialize the Grammian matrix

for i=1:length(basiscenter)                                 % Double for loop to assign components as defined
    for j=1:length(basiscenter)
        k_xi_xi(1+m*(i-1):m+m*(i-1),1+m*(j-1):m+m*(j-1))=knlfun(basiscenter(:,i),basiscenter(:,j));
    end
end
cond_K=cond(k_xi_xi);                                       % Condition number of the Grammian (big K) matrix
inv_k_xi_xi=inv(k_xi_xi);                                   % inverse of the Grammian (big K) matrix

% Offline calculation of maximum power function value
Np=2*N;                                                    % Grid refinement subdivision number
xp=linspace(rangeX(1),rangeX(2),Np);                        % X coordinate of the test grid
yp=linspace(rangeY(1),rangeY(2),Np);                        % Y coordinate of the test grid
zp=linspace(rangeZ(1),rangeZ(2),Np);                        % Y coordinate of the test grid

[Xp,Yp,Zp]=meshgrid(xp,yp,zp);        % creating meshgrid

testpoints=[Xp(:),Yp(:),Zp(:)]';                       % Assigning test points

% Power function value calculation
zp=zeros(1,length(testpoints));                                          % Initialize the storage of power function values over the test grid
knl_xp=zeros(m*N,m);                                          % Kernel evaluation vector initialization
for i=1:length(testpoints)                                  % for loop to calculate power function at each test point
    for k=1:length(basiscenter)
        knl_xp(1+m*(k-1):m+m*(k-1),:)=knlfun(testpoints(:,i),basiscenter(:,k));   % Kernel evaluation vector at each test point
    end
    zp(i)=max(diag(sqrt(knlfun(testpoints(:,i),testpoints(:,i))...
        -knl_xp'*(k_xi_xi\knl_xp))));                     % Power function value at each test point
end
pwfmax=max(zp,[],"all");                                    % Finding the maximum power function value over the domain

%% Export the desired matrices to a json file
fid=fopen('RKHS_Matrices.json', 'w');
for i=1:3
    center_json=jsonencode(basiscenter(i,:));
    fprintf(fid, '%s,\n', center_json);
end
fprintf(fid, '\n');
for i=1:length(inv_k_xi_xi)
    inv_k_xi_xi_json=jsonencode(round(inv_k_xi_xi(i,:),6));
    fprintf(fid, '%s,\n', inv_k_xi_xi_json);
end
fclose(fid);