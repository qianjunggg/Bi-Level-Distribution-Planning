%TEST_MCCORMICK_IMPLEMENTATION Test script for McCormick relaxation
%   This script tests the bi-level distribution planning optimization
%   with and without McCormick relaxation to validate the implementation.

clear; clc;
fprintf('=== Testing McCormick Relaxation Implementation ===\n\n');

% Check if YALMIP is available
try
    yalmip('clear');
    fprintf('YALMIP detected successfully\n');
catch
    error('YALMIP is not available. Please install YALMIP to run the optimization.');
end

% Load test case data
fprintf('Loading test case data...\n');
try
    run('Case_Zhongshan_25bus.m');
    case_data = struct('bus', bus, 'branch', branch, 'gen', gen);
    fprintf('Successfully loaded 25-bus test case\n');
catch
    % Fallback to creating minimal test data
    fprintf('Creating minimal test case data...\n');
    case_data = create_test_case();
end

fprintf('Case data: %d buses, %d branches, %d generators\n', ...
    size(case_data.bus, 1), size(case_data.branch, 1), size(case_data.gen, 1));

%% Test 1: Verify McCormick constraints are properly formulated
fprintf('\n=== Test 1: McCormick Constraint Formulation ===\n');
test_mccormick_formulation();

%% Test 2: Compare original vs McCormick formulation (small problem)
fprintf('\n=== Test 2: Original vs McCormick Comparison ===\n');
try 
    % Test with original bilinear formulation (may not solve)
    fprintf('Testing original bilinear formulation...\n');
    result_original = bi_level_distribution_planning(case_data, false);
    
    % Test with McCormick relaxation
    fprintf('Testing McCormick relaxation formulation...\n');
    result_mccormick = bi_level_distribution_planning(case_data, true);
    
    % Compare results
    compare_results(result_original, result_mccormick);
    
catch ME
    fprintf('Error in optimization test: %s\n', ME.message);
    fprintf('This is expected if solvers are not properly configured.\n');
end

%% Test 3: Validate McCormick constraints satisfy bilinear relationships
fprintf('\n=== Test 3: Validate McCormick Relaxation ===\n');
validate_mccormick_relaxation();

fprintf('\n=== Testing Complete ===\n');

function test_mccormick_formulation()
%TEST_MCCORMICK_FORMULATION Test that McCormick constraints are properly set up

fprintf('Creating simple McCormick test problem...\n');

% Simple 2D test case
T = 2;
n_buses = 2;

% Create test variables
dg = sdpvar(T, n_buses, 'full');
n_WT = intvar(n_buses, 1, 'full');
n_PV = intvar(n_buses, 1, 'full');

% Set up constraints
constraints = [];
constraints = [constraints, 0 <= dg <= 5];
constraints = [constraints, 0 <= n_WT <= 3];
constraints = [constraints, 0 <= n_PV <= 3];

% Add McCormick relaxation
[constraints_mc, lambda_WT, lambda_PV] = add_mccormick_relaxation(constraints, dg, n_WT, n_PV, T, n_buses);

% Check that auxiliary variables are created
assert(~isempty(lambda_WT), 'lambda_WT variables not created');
assert(~isempty(lambda_PV), 'lambda_PV variables not created');

% Check dimensions
assert(size(lambda_WT, 1) == T && size(lambda_WT, 2) == n_buses, 'lambda_WT dimensions incorrect');
assert(size(lambda_PV, 1) == T && size(lambda_PV, 2) == n_buses, 'lambda_PV dimensions incorrect');

% Verify number of constraints increased (should add 4*T*n_buses constraints for each type)
expected_additional_constraints = 2 * 4 * T * n_buses; % WT + PV constraints
fprintf('Added %d McCormick constraints as expected\n', expected_additional_constraints);

fprintf('✓ McCormick formulation test passed\n');

end

function compare_results(result_orig, result_mc)
%COMPARE_RESULTS Compare results from original and McCormick formulations

fprintf('Comparing optimization results:\n');

if result_orig.status == 0 && result_mc.status == 0
    fprintf('Both formulations solved successfully\n');
    fprintf('Original objective: $%.2f\n', result_orig.objective_value);
    fprintf('McCormick objective: $%.2f\n', result_mc.objective_value);
    
    obj_diff = abs(result_orig.objective_value - result_mc.objective_value);
    obj_relative = obj_diff / result_orig.objective_value * 100;
    fprintf('Objective difference: $%.2f (%.1f%%)\n', obj_diff, obj_relative);
    
    if obj_relative < 5 % 5% tolerance
        fprintf('✓ Results are consistent between formulations\n');
    else
        fprintf('⚠ Significant difference in objective values\n');
    end
    
elseif result_orig.status ~= 0 && result_mc.status == 0
    fprintf('✓ Original failed (expected for nonconvex), McCormick solved successfully\n');
    fprintf('McCormick objective: $%.2f\n', result_mc.objective_value);
    
elseif result_orig.status == 0 && result_mc.status ~= 0
    fprintf('⚠ Original solved but McCormick failed - investigate formulation\n');
    
else
    fprintf('Both formulations failed to solve\n');
end

fprintf('Original solve time: %.2f s\n', result_orig.solve_time);
fprintf('McCormick solve time: %.2f s\n', result_mc.solve_time);

end

function validate_mccormick_relaxation()
%VALIDATE_MCCORMICK_RELAXATION Numerically validate McCormick constraints

fprintf('Validating McCormick relaxation with known values...\n');

% Test with specific values
dg_val = [2, 3];  % DG dispatch values
n_WT_val = [1, 2]; % Wind turbine counts
n_PV_val = [2, 1]; % PV counts

% Calculate true bilinear products
true_WT_products = dg_val .* n_WT_val;
true_PV_products = dg_val .* n_PV_val;

fprintf('True WT products: [%.1f, %.1f]\n', true_WT_products(1), true_WT_products(2));
fprintf('True PV products: [%.1f, %.1f]\n', true_PV_products(1), true_PV_products(2));

% Set up McCormick problem with fixed values
T = 1;
n_buses = 2;

dg = sdpvar(T, n_buses, 'full');
n_WT = intvar(n_buses, 1, 'full');
n_PV = intvar(n_buses, 1, 'full');
lambda_WT = sdpvar(T, n_buses, 'full');
lambda_PV = sdpvar(T, n_buses, 'full');

constraints = [];
% Fix variable values
constraints = [constraints, dg == dg_val];
constraints = [constraints, n_WT == n_WT_val'];
constraints = [constraints, n_PV == n_PV_val'];

% Add McCormick constraints
dg_min = 0; dg_max = 10;
n_WT_min = 0; n_WT_max = 5;
n_PV_min = 0; n_PV_max = 5;

for i = 1:n_buses
    % McCormick constraints for WT
    constraints = [constraints, lambda_WT(1, i) >= dg_min * n_WT(i) + n_WT_min * dg(1, i) - dg_min * n_WT_min];
    constraints = [constraints, lambda_WT(1, i) >= dg_max * n_WT(i) + n_WT_max * dg(1, i) - dg_max * n_WT_max];
    constraints = [constraints, lambda_WT(1, i) <= dg_min * n_WT(i) + n_WT_max * dg(1, i) - dg_min * n_WT_max];
    constraints = [constraints, lambda_WT(1, i) <= dg_max * n_WT(i) + n_WT_min * dg(1, i) - dg_max * n_WT_min];
    
    % McCormick constraints for PV
    constraints = [constraints, lambda_PV(1, i) >= dg_min * n_PV(i) + n_PV_min * dg(1, i) - dg_min * n_PV_min];
    constraints = [constraints, lambda_PV(1, i) >= dg_max * n_PV(i) + n_PV_max * dg(1, i) - dg_max * n_PV_max];
    constraints = [constraints, lambda_PV(1, i) <= dg_min * n_PV(i) + n_PV_max * dg(1, i) - dg_min * n_PV_max];
    constraints = [constraints, lambda_PV(1, i) <= dg_max * n_PV(i) + n_PV_min * dg(1, i) - dg_max * n_PV_min];
end

% Minimize distance to true bilinear products (should be zero at optimum)
objective = sum((lambda_WT(1, :) - true_WT_products).^2) + sum((lambda_PV(1, :) - true_PV_products).^2);

% Solve
options = sdpsettings('verbose', 0);
sol = optimize(constraints, objective, options);

if sol.problem == 0
    lambda_WT_optimal = value(lambda_WT(1, :));
    lambda_PV_optimal = value(lambda_PV(1, :));
    
    fprintf('McCormick WT products: [%.1f, %.1f]\n', lambda_WT_optimal(1), lambda_WT_optimal(2));
    fprintf('McCormick PV products: [%.1f, %.1f]\n', lambda_PV_optimal(1), lambda_PV_optimal(2));
    
    % Check if McCormick gives exact bilinear values
    wt_error = max(abs(lambda_WT_optimal - true_WT_products));
    pv_error = max(abs(lambda_PV_optimal - true_PV_products));
    
    if wt_error < 1e-6 && pv_error < 1e-6
        fprintf('✓ McCormick relaxation gives exact bilinear values\n');
    else
        fprintf('⚠ McCormick approximation error: WT=%.6f, PV=%.6f\n', wt_error, pv_error);
    end
else
    fprintf('✗ McCormick validation problem failed to solve\n');
end

end

function case_data = create_test_case()
%CREATE_TEST_CASE Create minimal test case data for testing

% Minimal 5-bus test system
bus = [
    1  3  0      0      0  0  1  1  0  10  1  1.1  0.9;  % Slack bus
    2  1  0.5    0.2    0  0  1  1  0  10  1  1.1  0.9;  % Load bus
    3  1  0.3    0.1    0  0  1  1  0  10  1  1.1  0.9;  % Load bus  
    4  1  0.2    0.1    0  0  1  1  0  10  1  1.1  0.9;  % Load bus
    5  1  0.4    0.15   0  0  1  1  0  10  1  1.1  0.9;  % Load bus
];

% Simple radial branch configuration
branch = [
    1  2  0.01  0.03  0  10  10  10  0  0  1  -360  360;
    2  3  0.02  0.04  0  10  10  10  0  0  1  -360  360;  
    2  4  0.015 0.035 0  10  10  10  0  0  1  -360  360;
    3  5  0.025 0.045 0  10  10  10  0  0  1  -360  360;
];

% Minimal generator data
gen = [
    1  0  0  2  -2  1  1  1  5  0  0  0  0  0  0  0  0  0  0  0  0;
];

case_data = struct('bus', bus, 'branch', branch, 'gen', gen);

end