%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025 Giri M. Kumar, Andrea L'Afflitto. All rights reserved.                                                        
%                                                                             
% Redistribution and use in source and binary forms, with or without          
% modification, are permitted provided that the following conditions are met: 
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
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE   
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE  
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE   
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR         
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF        
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS    
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN     
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)     
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE  
% POSSIBILITY OF SUCH DAMAGE.                                                 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File:       trajectory_generator.m                                                   
% Author:     Mattia Gramuglia
% Date:       June 20, 2025
% For info:   Andrea L'Afflitto                                               
%             a.lafflitto@vt.edu                                              
%                                                                             
% Description: This live script is used to generate trajectories from
%              points.
%
% GitHub: https://github.com/girimugundankumar/qrbp-unwinding-constraints.git   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% NOTE: Requries the UAV Toolbox

%% -------------------------------------------------------- Read the yaml file
% Setup the matlab environment
close all
clear all
clc

% Add path to the read_input function
addpath('functions/');

% Add path to the yaml library
addpath(genpath('../../libraries/third-party/yamlmatlab/'));
addpath('../../libraries/third-party/yamlmatlab/');

% Injest the yaml file - Get the points, time, alphafactor and name.
traj = read_input('input/datafile.yaml');

%% ----------------------------------------------------- Compute the trajectory
% Minimum jerk trajectory
% Output Arguments: q - Positions of trajectory [ n-by-m matrix ]
%                   qd - Velocities of trajectory [ n-by-m matrix ]
%                   qdd - Accelerations of trajectory [ n-by-m matrix ]
%                   qddd - Jerks of trajectory [ n-by-m matrix ]
%                   pp - Piecewise polynomial [ structure ]
%                   tPoints - Time points for waypoints [ p-element row vector ]
%                   tSamples - Time samples [ m-element row vector ]
[traj.pos,traj.vel,traj.acc,traj.jerk,traj.pp,traj.timepoints,traj.tsamples] = ...
                         minjerkpolytraj(traj.wpts, traj.tpts, traj.numsamples);

% TRAJECTORY POSITION 
% Assigning the polynomial coefficients to the designated variables
[position.coef.x,...
 position.coef.y,...
 position.coef.z] = PolyCoefReshape(traj.pp.coefs);

% TRAJECTORY VELOCITY 
% Assigning the polynomial coefficients to the designated variables
velocity.coef.x = PolyDerMatrix(position.coef.x);
velocity.coef.y = PolyDerMatrix(position.coef.y);
velocity.coef.z = PolyDerMatrix(position.coef.z);

% TRAJECTORY ACCELERATION 
% Assigning the polynomial coefficients to the designated variables
acceleration.coef.x = PolyDerMatrix(velocity.coef.x);
acceleration.coef.y = PolyDerMatrix(velocity.coef.y);
acceleration.coef.z = PolyDerMatrix(velocity.coef.z);

% TRAJECTORY JERK 
% Assigning the polynomial coefficients to the designated variables
jerk.coef.x = PolyDerMatrix(acceleration.coef.x);
jerk.coef.y = PolyDerMatrix(acceleration.coef.y);
jerk.coef.z = PolyDerMatrix(acceleration.coef.z);

%% ----------------------------------------------- Visualize the generated path
% Plotting trajectory in 3D
animate_3d_trajectory(traj);

%% ------------------------------------------------- Compute the yaw trajectory
% Obtain the time vector with the tsteps from the input data file
t = traj.tpts(1):traj.step:traj.tpts(end);

for ii = 1:length(t)

% Adjust the polynomial and segemnt it 
[t_adjusted(ii),segment(ii)] = getSegmentIndexAndLocalTime(traj.tpts,t(ii));

position.x(ii) = polyval(position.coef.x(segment(ii),:), t_adjusted(ii));
position.y(ii) = polyval(position.coef.y(segment(ii),:), t_adjusted(ii));
position.z(ii) = polyval(position.coef.z(segment(ii),:), t_adjusted(ii));

velocity.x(ii) = polyval(velocity.coef.x(segment(ii),:), t_adjusted(ii));
velocity.y(ii) = polyval(velocity.coef.y(segment(ii),:), t_adjusted(ii));
velocity.z(ii) = polyval(velocity.coef.z(segment(ii),:), t_adjusted(ii));

velocity.norm2D(ii) = norm2D(velocity.coef.x(segment(ii),:),...
                             velocity.coef.y(segment(ii),:),...
                             t_adjusted(ii));

acceleration.x(ii) = polyval(acceleration.coef.x(segment(ii),:), t_adjusted(ii));
acceleration.y(ii) = polyval(acceleration.coef.y(segment(ii),:), t_adjusted(ii));
acceleration.z(ii) = polyval(acceleration.coef.z(segment(ii),:), t_adjusted(ii));

jerk.x(ii) = polyval(jerk.coef.x(segment(ii),:), t_adjusted(ii));
jerk.y(ii) = polyval(jerk.coef.y(segment(ii),:), t_adjusted(ii));
jerk.z(ii) = polyval(jerk.coef.z(segment(ii),:), t_adjusted(ii));

if (traj.set_yaw_zero)
    yaw(ii) = 0.0;
    yaw_dot(ii) = 0.0;
    yaw_dot_dot(ii) = 0.0;
else
    if t_adjusted(ii) == 0
        yaw(ii) = 0;
        yaw_dot(ii) = 0;
        yaw_dot_dot(ii) = 0;
    
    elseif t_adjusted(ii) > 0 & velocity.norm2D(ii) < 1e-2
        yaw(ii) = yaw(ii-1);
        yaw_dot(ii) = 0;
        yaw_dot_dot(ii) = 0;
    
    else
        [yaw(ii), yaw_dot(ii), yaw_dot_dot(ii)] = ...
        ComputeYawTrajectory(velocity.coef.x(segment(ii),:),...  
                             velocity.coef.y(segment(ii),:),...
                             acceleration.coef.x(segment(ii),:),...
                             acceleration.coef.y(segment(ii),:),...
                             jerk.coef.x(segment(ii),:),...
                             jerk.coef.y(segment(ii),:),...
                             t_adjusted(ii));
    end
end

end



%% ------------------------------- Visualize the generated path w/ heading data
% Plots Yaw, Yaw_dot, Yaw_dot_dot
animate_3d_traj_with_heading(t, position.x, position.y, position.z, ...
                             yaw, yaw_dot, yaw_dot_dot, traj.wpts);


%% ----------------------- Export it for the flightstack and chrono simulations
if strcmp(traj.codebase, 'cpp')

    json_struct.hover_time.seconds = traj.hover_time;
    json_struct.waypoints = traj.wpts;
    json_struct.waypoint_times = traj.tpts;
    json_struct.piecewise_polynomial_coefficients = traj.pp.coefs;

    structToJSONcpp(json_struct, traj);

end

%% ---------------------------------------------- Export it for the matlab sim
if strcmp(traj.codebase, 'matlab')

   output_for_matlab(t, position, velocity, acceleration, ...
                     yaw, yaw_dot, yaw_dot_dot, traj);

end

