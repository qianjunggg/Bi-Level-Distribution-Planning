#!/usr/bin/env python3
"""
Syntax validation for MATLAB McCormick relaxation implementation.
This script performs basic syntax and structure validation of the MATLAB code.
"""

import os
import re
import sys

def check_matlab_syntax(filename):
    """Basic MATLAB syntax checking"""
    print(f"\n=== Checking {filename} ===")
    
    if not os.path.exists(filename):
        print(f"ERROR: File {filename} not found")
        return False
        
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            content = f.read()
    except UnicodeDecodeError:
        with open(filename, 'r', encoding='latin1') as f:
            content = f.read()
        
    issues = []
    
    # Check for balanced brackets/parentheses
    brackets = {'(': ')', '[': ']', '{': '}'}
    stack = []
    
    for i, char in enumerate(content):
        if char in brackets:
            stack.append((char, i))
        elif char in brackets.values():
            if not stack:
                issues.append(f"Unmatched closing bracket '{char}' at position {i}")
            else:
                open_char, _ = stack.pop()
                if brackets[open_char] != char:
                    issues.append(f"Mismatched bracket pair at position {i}")
    
    if stack:
        for char, pos in stack:
            issues.append(f"Unmatched opening bracket '{char}' at position {pos}")
    
    # Check for common MATLAB syntax patterns
    lines = content.split('\n')
    for i, line in enumerate(lines, 1):
        line = line.strip()
        if not line or line.startswith('%'):
            continue
            
        # Check for proper function definitions
        if line.startswith('function'):
            if not re.match(r'function\s+.*=.*\(.*\)', line) and not re.match(r'function\s+\w+\(.*\)', line):
                issues.append(f"Line {i}: Potential function syntax issue: {line}")
        
        # Check for end statements
        if line == 'end' or line.startswith('end '):
            continue  # These are fine
            
        # Check for semicolon usage (warning only)
        if '=' in line and not line.endswith(';') and not line.endswith('...'):
            if not any(keyword in line for keyword in ['if', 'for', 'while', 'function', 'end']):
                print(f"Line {i}: Consider adding semicolon to suppress output: {line[:50]}...")
    
    # Check for required functions/structures
    required_patterns = [
        r'function.*bi_level_distribution_planning',
        r'function.*add_mccormick_relaxation', 
        r'lambda_WT\s*=\s*sdpvar',
        r'lambda_PV\s*=\s*sdpvar',
        r'constraints\s*=.*lambda_WT',
        r'constraints\s*=.*lambda_PV',
    ]
    
    for pattern in required_patterns:
        if not re.search(pattern, content):
            issues.append(f"Missing expected pattern: {pattern}")
    
    if issues:
        print(f"ISSUES FOUND ({len(issues)}):")
        for issue in issues[:10]:  # Show first 10 issues
            print(f"  - {issue}")
        if len(issues) > 10:
            print(f"  ... and {len(issues) - 10} more issues")
        return False
    else:
        print("✓ No syntax issues detected")
        return True

def check_file_structure():
    """Check if all required files are present with expected content"""
    print("\n=== Checking File Structure ===")
    
    required_files = {
        'bi_level_distribution_planning.m': ['function', 'mccormick', 'lambda_WT', 'lambda_PV'],
        'test_mccormick_implementation.m': ['test', 'validate', 'mccormick'],
        'example_usage.m': ['example', 'usage', 'demonstration'],
        'McCormick_Relaxation_README.md': ['McCormick', 'relaxation', 'bilinear'],
        'Case_Zhongshan_25bus.m': ['bus', 'branch', 'gen'],
    }
    
    all_good = True
    for filename, expected_content in required_files.items():
        if not os.path.exists(filename):
            print(f"✗ Missing file: {filename}")
            all_good = False
            continue
            
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                content = f.read().lower()
        except UnicodeDecodeError:
            try:
                with open(filename, 'r', encoding='latin1') as f:
                    content = f.read().lower()
            except:
                print(f"⚠ {filename}: Unable to read file (encoding issue)")
                continue
            
        missing_content = []
        for expected in expected_content:
            if expected.lower() not in content:
                missing_content.append(expected)
                
        if missing_content:
            print(f"⚠ {filename}: Missing expected content: {missing_content}")
        else:
            print(f"✓ {filename}: All expected content found")
    
    return all_good

def validate_mccormick_implementation():
    """Validate specific McCormick implementation details"""
    print("\n=== Validating McCormick Implementation ===")
    
    filename = 'bi_level_distribution_planning.m'
    if not os.path.exists(filename):
        print(f"ERROR: {filename} not found")
        return False
        
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            content = f.read()
    except UnicodeDecodeError:
        with open(filename, 'r', encoding='latin1') as f:
            content = f.read()
    
    # Check for McCormick constraint patterns
    mccormick_patterns = [
        r'lambda_WT.*>=.*dg_min.*n_WT.*dg_min.*n_WT_min',  # McCormick constraint 1
        r'lambda_WT.*>=.*dg_max.*n_WT.*dg_max.*n_WT_max',  # McCormick constraint 2  
        r'lambda_WT.*<=.*dg_min.*n_WT.*dg_min.*n_WT_max',  # McCormick constraint 3
        r'lambda_WT.*<=.*dg_max.*n_WT.*dg_max.*n_WT_min',  # McCormick constraint 4
        r'lambda_PV.*>=.*dg_min.*n_PV.*dg_min.*n_PV_min',  # PV constraints
    ]
    
    found_patterns = 0
    for pattern in mccormick_patterns:
        if re.search(pattern, content):
            found_patterns += 1
    
    print(f"Found {found_patterns}/{len(mccormick_patterns)} expected McCormick constraint patterns")
    
    # Check for auxiliary variable creation
    if 'lambda_WT = sdpvar' in content and 'lambda_PV = sdpvar' in content:
        print("✓ Auxiliary variables properly defined")
    else:
        print("✗ Auxiliary variables not properly defined")
        
    # Check for both formulations (original and McCormick)
    if 'use_mccormick' in content:
        print("✓ Support for both original and McCormick formulations")
    else:
        print("⚠ May not support both formulations")
        
    return found_patterns >= len(mccormick_patterns) // 2  # At least half should match

def main():
    """Main validation function"""
    print("MATLAB McCormick Relaxation Implementation Validation")
    print("=" * 55)
    
    os.chdir('/home/runner/work/Bi-Level-Distribution-Planning/Bi-Level-Distribution-Planning')
    
    # Check file structure
    structure_ok = check_file_structure()
    
    # Check main MATLAB files
    matlab_files = [
        'bi_level_distribution_planning.m',
        'test_mccormick_implementation.m', 
        'example_usage.m'
    ]
    
    syntax_ok = True
    for filename in matlab_files:
        if not check_matlab_syntax(filename):
            syntax_ok = False
    
    # Validate McCormick implementation
    mccormick_ok = validate_mccormick_implementation()
    
    # Overall assessment
    print("\n" + "=" * 55)
    print("VALIDATION SUMMARY")
    print("=" * 55)
    
    print(f"File structure: {'✓ PASS' if structure_ok else '✗ FAIL'}")
    print(f"Syntax checking: {'✓ PASS' if syntax_ok else '✗ FAIL'}")  
    print(f"McCormick implementation: {'✓ PASS' if mccormick_ok else '✗ FAIL'}")
    
    overall_status = structure_ok and syntax_ok and mccormick_ok
    print(f"\nOVERALL: {'✓ VALIDATION PASSED' if overall_status else '✗ VALIDATION FAILED'}")
    
    if overall_status:
        print("\nThe McCormick relaxation implementation appears to be correctly structured.")
        print("Manual testing with MATLAB/YALMIP is recommended for full validation.")
    else:
        print("\nPlease review the issues identified above.")
    
    return 0 if overall_status else 1

if __name__ == '__main__':
    sys.exit(main())