function [result] = bi_level_distribution_planning(case_data, use_mccormick)
%BI_LEVEL_DISTRIBUTION_PLANNING Solve bi-level distribution planning problem
%   This function implements a bi-level distribution planning optimization
%   with uncertain renewable generation. It includes bilinear terms that can
%   be linearized using McCormick relaxation.
%
%   Inputs:
%       case_data - struct containing bus, branch, and gen data
%       use_mccormick - boolean flag to enable McCormick relaxation (default: false)
%
%   Outputs:
%       result - optimization results structure

if nargin < 2
    use_mccormick = false;
end

% Load case data
if ischar(case_data)
    run(case_data);
    case_data = struct('bus', bus, 'branch', branch, 'gen', gen);
end

% Extract system parameters
nbus = size(case_data.bus, 1);
nbranch = size(case_data.branch, 1);
ngen = size(case_data.gen, 1);

% Time periods
T = 24; % 24 hours
% Number of scenarios for uncertainty
nscenarios = 10;

% DG installation parameters
n_candidate_buses = 10; % Number of candidate buses for DG installation
candidate_buses = [5, 10, 16, 19, 25, 38, 50, 56, 78, 90]; % Example candidate buses
n_WT_max = 5; % Maximum number of wind turbines per bus
n_PV_max = 5; % Maximum number of PV units per bus

% Wind and PV parameters
P_WT_nom = 1.5; % Nominal power of wind turbine (MW)
P_PV_nom = 1.0; % Nominal power of PV unit (MW)

% Uncertainty parameters (scenarios)
S_WT_0 = rand(nscenarios, n_candidate_buses); % Wind generation scenarios
S_PV_0 = rand(nscenarios, n_candidate_buses); % PV generation scenarios

% Reserve requirements
reserve_factor = 0.1; % 10% reserve requirement

fprintf('Setting up optimization problem...\n');
fprintf('System: %d buses, %d branches, %d generators\n', nbus, nbranch, ngen);
fprintf('Candidate DG buses: %d\n', n_candidate_buses);
fprintf('Time periods: %d\n', T);
fprintf('Uncertainty scenarios: %d\n', nscenarios);

% Create optimization variables using YALMIP
% Decision variables
n_WT = intvar(n_candidate_buses, 1, 'full'); % Number of wind turbines
n_PV = intvar(n_candidate_buses, 1, 'full'); % Number of PV units
dg = sdpvar(T, n_candidate_buses, 'full'); % DG dispatch decisions

% Power flow variables
P = sdpvar(T, nbus, 'full'); % Active power injection
Q = sdpvar(T, nbus, 'full'); % Reactive power injection
V = sdpvar(T, nbus, 'full'); % Voltage magnitude
theta = sdpvar(T, nbus, 'full'); % Voltage angle

% Reserve variables
R_up = sdpvar(T, n_candidate_buses, 'full'); % Upward reserve
R_down = sdpvar(T, n_candidate_buses, 'full'); % Downward reserve

% Cost parameters
cost_WT = 1000; % Cost per wind turbine
cost_PV = 800;  % Cost per PV unit
cost_energy = 50; % Energy cost
cost_reserve = 20; % Reserve cost

% Initialize constraints
constraints = [];

% DG installation constraints
constraints = [constraints, 0 <= n_WT <= n_WT_max];
constraints = [constraints, 0 <= n_PV <= n_PV_max];

% DG dispatch constraints
for i = 1:n_candidate_buses
    constraints = [constraints, 0 <= dg(:, i) <= P_WT_nom * n_WT(i) + P_PV_nom * n_PV(i)];
end

% Voltage constraints
constraints = [constraints, 0.9 <= V <= 1.1];

% Reserve constraints (this is where bilinear terms appear)
fprintf('Adding bilinear reserve constraints...\n');

% The bilinear terms appear in uncertainty constraints
% Original formulation (nonconvex):
% awinres = [-dg * n_WT' * S_WT_0, -dg * n_PV' * S_PV_0 ; 
%             dg * n_WT' * S_WT_0,  dg * n_PV' * S_PV_0];

if use_mccormick
    fprintf('Using McCormick relaxation for bilinear terms...\n');
    [constraints, lambda_WT, lambda_PV] = add_mccormick_relaxation(constraints, dg, n_WT, n_PV, T, n_candidate_buses);
    
    % Use auxiliary variables in uncertainty constraints
    for t = 1:T
        for s = 1:nscenarios
            % Reserve requirements with McCormick variables
            wind_reserve_up = sum(lambda_WT(t, :) .* S_WT_0(s, :));
            wind_reserve_down = sum(lambda_WT(t, :) .* S_WT_0(s, :));
            pv_reserve_up = sum(lambda_PV(t, :) .* S_PV_0(s, :));
            pv_reserve_down = sum(lambda_PV(t, :) .* S_PV_0(s, :));
            
            constraints = [constraints, R_up(t, :) >= reserve_factor * (wind_reserve_up + pv_reserve_up)];
            constraints = [constraints, R_down(t, :) >= reserve_factor * (wind_reserve_down + pv_reserve_down)];
        end
    end
else
    fprintf('Using original bilinear formulation (nonconvex)...\n');
    % Original bilinear constraints (nonconvex)
    for t = 1:T
        for s = 1:nscenarios
            % Direct bilinear terms
            wind_reserve_up = sum(dg(t, :) .* n_WT' .* S_WT_0(s, :));
            wind_reserve_down = sum(dg(t, :) .* n_WT' .* S_WT_0(s, :));
            pv_reserve_up = sum(dg(t, :) .* n_PV' .* S_PV_0(s, :));
            pv_reserve_down = sum(dg(t, :) .* n_PV' .* S_PV_0(s, :));
            
            constraints = [constraints, R_up(t, :) >= reserve_factor * (wind_reserve_up + pv_reserve_up)];
            constraints = [constraints, R_down(t, :) >= reserve_factor * (wind_reserve_down + pv_reserve_down)];
        end
    end
end

% Reserve capacity constraints
constraints = [constraints, R_up >= 0];
constraints = [constraints, R_down >= 0];

% Simple power balance constraints (simplified for demonstration)
for t = 1:T
    constraints = [constraints, sum(P(t, :)) == sum(case_data.bus(:, 3))]; % Power balance
end

% Objective function
installation_cost = cost_WT * sum(n_WT) + cost_PV * sum(n_PV);
operation_cost = cost_energy * sum(sum(dg));
reserve_cost = cost_reserve * sum(sum(R_up + R_down));

objective = installation_cost + operation_cost + reserve_cost;

% Solve the optimization problem
fprintf('Solving optimization problem...\n');
if use_mccormick
    fprintf('Problem type: Mixed Integer Linear Program (MILP)\n');
else
    fprintf('Problem type: Mixed Integer Nonlinear Program (MINLP) - Nonconvex\n');
end

options = sdpsettings('verbose', 1, 'solver', 'gurobi');
if ~use_mccormick
    options = sdpsettings('verbose', 1, 'solver', 'bonmin'); % For nonconvex problems
end

sol = optimize(constraints, objective, options);

% Extract results
result = struct();
result.status = sol.problem;
result.solve_time = sol.solvertime;
result.objective_value = value(objective);
result.n_WT_optimal = value(n_WT);
result.n_PV_optimal = value(n_PV);
result.dg_optimal = value(dg);
result.installation_cost = value(installation_cost);
result.operation_cost = value(operation_cost);
result.reserve_cost = value(reserve_cost);

% Display results
fprintf('\n=== Optimization Results ===\n');
if sol.problem == 0
    fprintf('Status: Optimal solution found\n');
elseif sol.problem == 1
    fprintf('Status: Infeasible problem\n');
elseif sol.problem == 2
    fprintf('Status: Unbounded problem\n');
else
    fprintf('Status: Solver error (code: %d)\n', sol.problem);
end

fprintf('Solve time: %.2f seconds\n', sol.solvertime);
fprintf('Objective value: $%.2f\n', value(objective));
fprintf('Installation cost: $%.2f\n', value(installation_cost));
fprintf('Operation cost: $%.2f\n', value(operation_cost));
fprintf('Reserve cost: $%.2f\n', value(reserve_cost));

fprintf('\nOptimal DG installations:\n');
for i = 1:n_candidate_buses
    if value(n_WT(i)) > 0 || value(n_PV(i)) > 0
        fprintf('Bus %d: %d WT, %d PV\n', candidate_buses(i), value(n_WT(i)), value(n_PV(i)));
    end
end

end

function [constraints, lambda_WT, lambda_PV] = add_mccormick_relaxation(constraints, dg, n_WT, n_PV, T, n_candidate_buses)
%ADD_MCCORMICK_RELAXATION Add McCormick relaxation for bilinear terms
%   This function adds auxiliary variables and McCormick constraints to
%   linearize the bilinear terms dg(i) * n_WT(j) and dg(i) * n_PV(j)

fprintf('Adding McCormick relaxation variables and constraints...\n');

% Create auxiliary variables for bilinear terms
lambda_WT = sdpvar(T, n_candidate_buses, 'full'); % dg * n_WT
lambda_PV = sdpvar(T, n_candidate_buses, 'full'); % dg * n_PV

% Variable bounds (required for McCormick relaxation)
dg_min = 0;   % Minimum DG dispatch
dg_max = 10;  % Maximum DG dispatch (conservative upper bound)
n_WT_min = 0; % Minimum number of wind turbines
n_WT_max = 5; % Maximum number of wind turbines
n_PV_min = 0; % Minimum number of PV units  
n_PV_max = 5; % Maximum number of PV units

% McCormick relaxation constraints for dg * n_WT terms
fprintf('Adding McCormick constraints for wind turbine terms...\n');
for t = 1:T
    for i = 1:n_candidate_buses
        % Four McCormick constraints for lambda_WT(t,i) = dg(t,i) * n_WT(i)
        % Constraint 1: lambda >= dg_min * n_WT + n_WT_min * dg - dg_min * n_WT_min
        constraints = [constraints, lambda_WT(t, i) >= dg_min * n_WT(i) + n_WT_min * dg(t, i) - dg_min * n_WT_min];
        
        % Constraint 2: lambda >= dg_max * n_WT + n_WT_max * dg - dg_max * n_WT_max  
        constraints = [constraints, lambda_WT(t, i) >= dg_max * n_WT(i) + n_WT_max * dg(t, i) - dg_max * n_WT_max];
        
        % Constraint 3: lambda <= dg_min * n_WT + n_WT_max * dg - dg_min * n_WT_max
        constraints = [constraints, lambda_WT(t, i) <= dg_min * n_WT(i) + n_WT_max * dg(t, i) - dg_min * n_WT_max];
        
        % Constraint 4: lambda <= dg_max * n_WT + n_WT_min * dg - dg_max * n_WT_min
        constraints = [constraints, lambda_WT(t, i) <= dg_max * n_WT(i) + n_WT_min * dg(t, i) - dg_max * n_WT_min];
    end
end

% McCormick relaxation constraints for dg * n_PV terms
fprintf('Adding McCormick constraints for PV terms...\n');
for t = 1:T
    for i = 1:n_candidate_buses
        % Four McCormick constraints for lambda_PV(t,i) = dg(t,i) * n_PV(i)
        % Constraint 1: lambda >= dg_min * n_PV + n_PV_min * dg - dg_min * n_PV_min
        constraints = [constraints, lambda_PV(t, i) >= dg_min * n_PV(i) + n_PV_min * dg(t, i) - dg_min * n_PV_min];
        
        % Constraint 2: lambda >= dg_max * n_PV + n_PV_max * dg - dg_max * n_PV_max
        constraints = [constraints, lambda_PV(t, i) >= dg_max * n_PV(i) + n_PV_max * dg(t, i) - dg_max * n_PV_max];
        
        % Constraint 3: lambda <= dg_min * n_PV + n_PV_max * dg - dg_min * n_PV_max
        constraints = [constraints, lambda_PV(t, i) <= dg_min * n_PV(i) + n_PV_max * dg(t, i) - dg_min * n_PV_max];
        
        % Constraint 4: lambda <= dg_max * n_PV + n_PV_min * dg - dg_max * n_PV_min
        constraints = [constraints, lambda_PV(t, i) <= dg_max * n_PV(i) + n_PV_min * dg(t, i) - dg_max * n_PV_min];
    end
end

fprintf('McCormick relaxation complete: %d WT constraints, %d PV constraints\n', ...
    4*T*n_candidate_buses, 4*T*n_candidate_buses);

end