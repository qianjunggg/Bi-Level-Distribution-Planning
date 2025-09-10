# McCormick Relaxation for Bi-Level Distribution Planning

This implementation addresses the bilinear terms in the uncertainty constraints of the bi-level distribution planning optimization problem by applying McCormick relaxation to convert the nonconvex problem into a Mixed Integer Linear Program (MILP).

## Problem Description

The original optimization model contains bilinear terms in the uncertainty constraints:

```matlab
awinres = [-dg * n_WT' * S_WT_0, -dg * n_PV' * S_PV_0 ; 
            dg * n_WT' * S_WT_0,  dg * n_PV' * S_PV_0];
```

Where:
- `dg(t,i)`: Continuous DG dispatch variable at time `t`, bus `i`
- `n_WT(i)`: Integer number of wind turbines at bus `i`
- `n_PV(i)`: Integer number of PV units at bus `i`
- `S_WT_0(s,i)`: Wind generation scenario `s` at bus `i`
- `S_PV_0(s,i)`: PV generation scenario `s` at bus `i`

The bilinear products `dg(t,i) * n_WT(i)` and `dg(t,i) * n_PV(i)` make the problem nonconvex.

## McCormick Relaxation Solution

### 1. Auxiliary Variables

For each bilinear term, introduce auxiliary variables:
- `λ_WT(t,i)` to replace `dg(t,i) * n_WT(i)`
- `λ_PV(t,i)` to replace `dg(t,i) * n_PV(i)`

### 2. McCormick Constraints

For each bilinear term `xy` with bounds `x ∈ [x_min, x_max]` and `y ∈ [y_min, y_max]`, add four constraints:

```matlab
λ ≥ x_min * y + y_min * x - x_min * y_min
λ ≥ x_max * y + y_max * x - x_max * y_max  
λ ≤ x_min * y + y_max * x - x_min * y_max
λ ≤ x_max * y + y_min * x - x_max * y_min
```

### 3. Variable Bounds

Required bounds for McCormick relaxation:
- `dg_min ≤ dg(t,i) ≤ dg_max`
- `n_WT_min ≤ n_WT(i) ≤ n_WT_max`  
- `n_PV_min ≤ n_PV(i) ≤ n_PV_max`

## Implementation Files

### Core Files

1. **`bi_level_distribution_planning.m`**
   - Main optimization function
   - Implements both original bilinear and McCormick relaxation formulations
   - Includes `add_mccormick_relaxation()` helper function

2. **`test_mccormick_implementation.m`** 
   - Comprehensive test suite
   - Validates McCormick constraint formulation
   - Compares original vs relaxed formulations
   - Numerical validation of McCormick constraints

3. **`example_usage.m`**
   - Demonstration script showing usage
   - Explains the bilinear problem and solution approach
   - Shows benefits of McCormick relaxation

### Supporting Files

4. **`Case_Zhongshan_25bus.m`**
   - 25-bus test system data (bus, branch, generator)

5. **`ieee 141/case141.m`**
   - 141-bus test system data

## Usage

### Basic Usage

```matlab
% Load case data
run('Case_Zhongshan_25bus.m');
case_data = struct('bus', bus, 'branch', branch, 'gen', gen);

% Solve with McCormick relaxation (recommended)
result = bi_level_distribution_planning(case_data, true);

% Solve original bilinear formulation (may fail)
result_orig = bi_level_distribution_planning(case_data, false);
```

### Testing

```matlab
% Run comprehensive tests
test_mccormick_implementation();

% Run example demonstration  
example_usage();
```

## Technical Details

### McCormick Constraint Generation

For each time period `t` and candidate bus `i`:

**Wind Turbine Terms (`dg * n_WT`):**
```matlab
λ_WT(t,i) ≥ dg_min * n_WT(i) + n_WT_min * dg(t,i) - dg_min * n_WT_min
λ_WT(t,i) ≥ dg_max * n_WT(i) + n_WT_max * dg(t,i) - dg_max * n_WT_max
λ_WT(t,i) ≤ dg_min * n_WT(i) + n_WT_max * dg(t,i) - dg_min * n_WT_max  
λ_WT(t,i) ≤ dg_max * n_WT(i) + n_WT_min * dg(t,i) - dg_max * n_WT_min
```

**PV Terms (`dg * n_PV`):** (Similar structure)

### Constraint Count

For a system with:
- `T` time periods
- `N` candidate DG buses

Total McCormick constraints added:
- Wind turbine constraints: `4 × T × N`
- PV constraints: `4 × T × N`  
- **Total: `8 × T × N` constraints**

Example: T=24 hours, N=10 buses → 1,920 additional constraints

## Benefits of McCormick Relaxation

1. **Convexity**: Converts nonconvex MINLP to convex MILP
2. **Global Optimality**: Guarantees finding global optimum (within relaxation gap)
3. **Solver Reliability**: Works with standard MILP solvers (Gurobi, CPLEX, etc.)
4. **Scalability**: Linear growth in constraint count
5. **Robustness**: Avoids local optima issues of nonconvex solvers

## Requirements

### Software Dependencies

- **MATLAB** R2016b or later
- **YALMIP** optimization modeling toolbox
- **MILP Solver** (one of):
  - Gurobi (recommended)
  - CPLEX
  - MOSEK
  - Free alternatives: GLPK, CBC

### Installation

1. Install YALMIP:
   ```matlab
   % Download from https://yalmip.github.io/
   addpath('path/to/yalmip')
   ```

2. Install solver (e.g., Gurobi):
   ```matlab
   % Follow solver-specific installation instructions
   ```

3. Verify installation:
   ```matlab
   yalmip('version')
   yalmip('solver')
   ```

## Validation

The implementation includes several validation mechanisms:

1. **Constraint Formulation Test**: Verifies McCormick constraints are properly set up
2. **Numerical Validation**: Confirms McCormick variables equal true bilinear products when variables are fixed
3. **Comparison Test**: Compares results between original and relaxed formulations
4. **Solver Status Check**: Ensures optimization completes successfully

## Performance Notes

- **Problem Size**: McCormick relaxation adds many constraints but maintains linearity
- **Solver Selection**: Use commercial MILP solvers for best performance on large problems
- **Bounds Tightening**: Tighter variable bounds improve McCormick relaxation quality
- **Preprocessing**: Some solvers automatically tighten McCormick constraints

## References

1. McCormick, G.P. (1976). "Computability of global solutions to factorable nonconvex programs: Part I — Convex underestimating problems"
2. Belotti, P. et al. (2009). "Mixed-integer nonlinear optimization"
3. Hijazi, H. et al. (2017). "Convex quadratic relaxations for mixed-integer nonlinear programs in power systems"

## Troubleshooting

### Common Issues

1. **"YALMIP not found"**
   - Install YALMIP from https://yalmip.github.io/

2. **"No suitable solver"**  
   - Install a MILP solver (Gurobi recommended)

3. **"Infeasible problem"**
   - Check variable bounds are reasonable
   - Verify case data is valid

4. **"Slow convergence"**
   - Use commercial solver (Gurobi/CPLEX)
   - Tighten variable bounds
   - Reduce problem size for testing

### Debug Mode

Enable detailed solver output:
```matlab
options = sdpsettings('verbose', 1, 'solver', 'gurobi');
result = bi_level_distribution_planning(case_data, true, options);
```