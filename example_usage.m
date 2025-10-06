%EXAMPLE_USAGE Example usage of bi-level distribution planning with McCormick relaxation
%   This script demonstrates how to use the McCormick relaxation implementation
%   for solving bi-level distribution planning problems.

clear; clc;
fprintf('=== Bi-Level Distribution Planning Example ===\n\n');

%% Setup
% Check if YALMIP is available
try
    yalmip('clear');
    fprintf('OK YALMIP is available\n');
catch
    fprintf('X YALMIP is not available\n');
    fprintf('Please install YALMIP from https://yalmip.github.io/\n');
    return;
end

% Load case data
fprintf('Loading case data...\n');
try
    run('Case_Zhongshan_25bus.m');
    case_data = struct('bus', bus, 'branch', branch, 'gen', gen);
    fprintf('OK Loaded Zhongshan 25-bus case\n');
catch
    fprintf('WARNING Could not load 25-bus case, using simplified test case\n');
    case_data = create_simple_case();
end

%% Demonstrate the bilinear problem
fprintf('\n=== Problem Description ===\n');
fprintf('This problem contains bilinear terms in uncertainty constraints:\n');
fprintf('  awinres = [-dg * n_WT'''' * S_WT_0, -dg * n_PV'''' * S_PV_0;\n');
fprintf('              dg * n_WT'''' * S_WT_0,  dg * n_PV'''' * S_PV_0]\n\n');

fprintf('Where:\n');
fprintf('  dg(t,i)   = DG dispatch at time t, bus i (continuous variable)\n');
fprintf('  n_WT(i)   = Number of wind turbines at bus i (integer variable)\n');  
fprintf('  n_PV(i)   = Number of PV units at bus i (integer variable)\n');
fprintf('  S_WT_0(s,i) = Wind generation scenario s at bus i\n');
fprintf('  S_PV_0(s,i) = PV generation scenario s at bus i\n\n');

fprintf('The bilinear terms dg(t,i) * n_WT(i) and dg(t,i) * n_PV(i)\n');
fprintf('make the problem nonconvex and difficult to solve.\n\n');

%% Solution approach
fprintf('=== Solution Approach: McCormick Relaxation ===\n');
fprintf('1. Introduce auxiliary variables:\n');
fprintf('   lambda_WT(t,i) approx dg(t,i) * n_WT(i)\n');
fprintf('   lambda_PV(t,i) approx dg(t,i) * n_PV(i)\n\n');

fprintf('2. Add McCormick constraints (4 per bilinear term):\n');
fprintf('   lambda >= dg_min * n + n_min * dg - dg_min * n_min\n');
fprintf('   lambda >= dg_max * n + n_max * dg - dg_max * n_max\n');
fprintf('   lambda <= dg_min * n + n_max * dg - dg_min * n_max\n');
fprintf('   lambda <= dg_max * n + n_min * dg - dg_max * n_min\n\n');

fprintf('3. Replace bilinear terms with auxiliary variables\n');
fprintf('4. Problem becomes Mixed Integer Linear Program (MILP)\n\n');

%% Solve with McCormick relaxation
fprintf('=== Solving with McCormick Relaxation ===\n');
try
    tic;
    result = bi_level_distribution_planning(case_data, true);
    solve_time = toc;
    
    fprintf('Solution completed in %.2f seconds\n', solve_time);
    
    if result.status == 0
        fprintf('OK Optimal solution found!\n\n');
        
        % Display key results
        fprintf('=== Results Summary ===\n');
        fprintf('Total cost: $%.2f\n', result.objective_value);
        fprintf('  Installation cost: $%.2f\n', result.installation_cost);
        fprintf('  Operation cost: $%.2f\n', result.operation_cost);  
        fprintf('  Reserve cost: $%.2f\n', result.reserve_cost);
        
        fprintf('\nOptimal DG Installations:\n');
        total_wt = sum(result.n_WT_optimal);
        total_pv = sum(result.n_PV_optimal);
        fprintf('  Total Wind Turbines: %d\n', total_wt);
        fprintf('  Total PV Units: %d\n', total_pv);
        
        if any(result.n_WT_optimal > 0) || any(result.n_PV_optimal > 0)
            fprintf('\nDetailed installations by bus:\n');
            for i = 1:length(result.n_WT_optimal)
                if result.n_WT_optimal(i) > 0 || result.n_PV_optimal(i) > 0
                    fprintf('  Bus %2d: %d WT, %d PV\n', i, ...
                        result.n_WT_optimal(i), result.n_PV_optimal(i));
                end
            end
        end
        
    else
        fprintf('X Solver failed with status code: %d\n', result.status);
    end
    
catch ME
    fprintf('X Error during optimization: %s\n', ME.message);
    fprintf('This may indicate missing solvers or configuration issues.\n');
end

%% Comparison without McCormick (demonstration)
fprintf('\n=== Comparison: Original Bilinear Formulation ===\n');
fprintf('Attempting to solve original bilinear problem (may fail)...\n');

try
    tic;
    result_orig = bi_level_distribution_planning(case_data, false);
    solve_time_orig = toc;
    
    if result_orig.status == 0
        fprintf('OK Original formulation also solved (%.2f seconds)\n', solve_time_orig);
        fprintf('Original objective: $%.2f\n', result_orig.objective_value);
        
        if exist('result', 'var') && result.status == 0
            obj_diff = abs(result.objective_value - result_orig.objective_value);
            fprintf('Difference from McCormick: $%.2f\n', obj_diff);
        end
    else
        fprintf('X Original formulation failed (expected for nonconvex problems)\n');
        fprintf('This demonstrates why McCormick relaxation is needed.\n');
    end
    
catch ME
    fprintf('X Original formulation failed: %s\n', ME.message);
    fprintf('This is expected - nonconvex problems are difficult to solve.\n');
end

%% Benefits summary
fprintf('\n=== McCormick Relaxation Benefits ===\n');
fprintf('OK Converts nonconvex MINLP to convex MILP\n');
fprintf('OK Guarantees global optimality (within relaxation gap)\n');
fprintf('OK Reliable solver performance\n');  
fprintf('OK Scalable to larger problems\n');
fprintf('OK Widely supported by commercial solvers\n\n');

fprintf('=== Example Complete ===\n');

function case_data = create_simple_case()
%CREATE_SIMPLE_CASE Create a simple test case for demonstration

bus = [
    1  3  0.0   0.0   0  0  1  1  0  10  1  1.1  0.9;  % Slack
    2  1  1.0   0.3   0  0  1  1  0  10  1  1.1  0.9;  % Load  
    3  1  0.8   0.2   0  0  1  1  0  10  1  1.1  0.9;  % Load
    4  1  0.6   0.2   0  0  1  1  0  10  1  1.1  0.9;  % Load
    5  1  0.4   0.1   0  0  1  1  0  10  1  1.1  0.9;  % Load
];

branch = [
    1  2  0.02  0.06  0  10  10  10  0  0  1  -360  360;
    2  3  0.03  0.07  0  10  10  10  0  0  1  -360  360;
    2  4  0.02  0.05  0  10  10  10  0  0  1  -360  360; 
    4  5  0.04  0.08  0  10  10  10  0  0  1  -360  360;
];

gen = [
    1  0  0  5  -5  1  1  1  10  0  0  0  0  0  0  0  0  0  0  0  0;
];

case_data = struct('bus', bus, 'branch', branch, 'gen', gen);

end