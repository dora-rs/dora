#!/usr/bin/env python3
"""
Automated script to fix linting issues in the codebase using ruff.

This script implements an iterative approach to fix linting issues by category,
allowing for controlled, incremental improvements to the codebase.
"""

import argparse
import subprocess
import sys
import re
from pathlib import Path
from typing import List, Optional, Set, Tuple


def run_command(cmd: List[str]) -> Tuple[str, str, int]:
    """Run a shell command and return stdout, stderr, and return code."""
    process = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    stdout, stderr = process.communicate()
    return stdout, stderr, process.returncode


def get_rule_categories() -> List[str]:
    """Get all rule categories from ruff."""
    # Run ruff check with statistics to get categories
    stdout, _, _ = run_command(["ruff", "check", ".", "--select", "ALL", "--statistics"])
    
    # Extract categories from output (e.g., E501, F401, W291)
    categories = set()
    pattern = r'[A-Z]+\d+'  # Match rule codes like E501, F401, etc.
    
    for line in stdout.splitlines():
        matches = re.findall(pattern, line)
        for match in matches:
            # Extract just the letter prefix (e.g., E, F, W)
            category = re.match(r'[A-Z]+', match)
            if category:
                categories.add(category.group())
    
    # If no categories found, use a default set of common categories
    if not categories:
        categories = {"E", "F", "W", "I", "N", "D", "UP", "B", "C", "T", "COM", "S"}
    
    return sorted(list(categories))


def get_fixable_rules() -> Set[str]:
    """Get all fixable rules from ruff."""
    # For now, we'll assume all rules are potentially fixable
    # In a real implementation, you might want to query ruff for this information
    return set()


def count_issues(category: Optional[str] = None) -> int:
    """Count the number of linting issues for a specific category or all categories."""
    cmd = ["ruff", "check", "."]
    if category:
        cmd.extend(["--select", category])
    
    stdout, _, _ = run_command(cmd)
    # Count lines but exclude warning lines about incompatible rules
    count = 0
    for line in stdout.splitlines():
        if not line.startswith("warning:") and re.search(r'[A-Z]+\d+', line):
            count += 1
    return count


def fix_category(category: str, dry_run: bool = False) -> int:
    """Fix linting issues for a specific category."""
    # First, get a list of files with issues in this category
    cmd_check = ["ruff", "check", ".", "--select", category, "--format", "json"]
    stdout_check, _, _ = run_command(cmd_check)
    
    # If no output or invalid JSON, return early
    if not stdout_check or stdout_check.strip() == "[]":
        return 0
    
    try:
        import json
        issues = json.loads(stdout_check)
    except json.JSONDecodeError:
        print(f"Error parsing ruff output for category {category}", file=sys.stderr)
        return count_issues(category)
    
    # Extract unique file paths
    files_with_issues = set()
    for issue in issues:
        if "filename" in issue:
            files_with_issues.add(issue["filename"])
    
    # Fix issues file by file
    fixed_count = 0
    for file_path in files_with_issues:
        cmd_fix = ["ruff", "check", file_path, "--select", category]
        if not dry_run:
            cmd_fix.append("--fix")
        
        before_stdout, _, _ = run_command(["ruff", "check", file_path, "--select", category])
        before_count = len([line for line in before_stdout.splitlines() 
                           if not line.startswith("warning:") and re.search(r'[A-Z]+\d+', line)])
        
        # Run the fix command
        _, _, _ = run_command(cmd_fix)
        
        # Check how many issues remain
        after_stdout, _, _ = run_command(["ruff", "check", file_path, "--select", category])
        after_count = len([line for line in after_stdout.splitlines() 
                          if not line.startswith("warning:") and re.search(r'[A-Z]+\d+', line)])
        
        fixed_count += (before_count - after_count)
    
    # Return the number of remaining issues
    return count_issues(category)


def main():
    """Main function to run the linting fix script."""
    parser = argparse.ArgumentParser(description="Fix linting issues in the codebase using ruff")
    parser.add_argument(
        "--category", 
        help="Specific rule category to fix (e.g., E, F, W). If not provided, all categories will be processed."
    )
    parser.add_argument(
        "--dry-run", 
        action="store_true", 
        help="Show what would be fixed without making changes"
    )
    parser.add_argument(
        "--list-categories", 
        action="store_true", 
        help="List all available rule categories and exit"
    )
    args = parser.parse_args()

    # Check if ruff is installed
    _, _, return_code = run_command(["ruff", "--version"])
    if return_code != 0:
        print("Error: ruff is not installed. Please install it with 'pip install ruff'", file=sys.stderr)
        sys.exit(1)

    # List categories if requested
    if args.list_categories:
        categories = get_rule_categories()
        print("Available rule categories:")
        for category in categories:
            print(f"  {category}")
        sys.exit(0)

    # Get fixable rules
    fixable_rules = get_fixable_rules()
    
    # Process a single category if specified
    if args.category:
        before_count = count_issues(args.category)
        if before_count == 0:
            print(f"No issues found for category {args.category}")
            sys.exit(0)
        
        print(f"Fixing {before_count} issues for category {args.category}...")
        remaining = fix_category(args.category, args.dry_run)
        fixed = before_count - remaining
        
        if args.dry_run:
            print(f"Would fix {fixed} issues (dry run)")
        else:
            print(f"Fixed {fixed} issues, {remaining} remaining")
        
        sys.exit(0)

    # Process all categories
    categories = get_rule_categories()
    total_before = count_issues()
    print(f"Found {total_before} total linting issues across {len(categories)} categories")
    
    if args.dry_run:
        print("Dry run mode - no changes will be made")
    
    for category in categories:
        before_count = count_issues(category)
        if before_count == 0:
            print(f"Category {category}: No issues found")
            continue
        
        print(f"Category {category}: Fixing {before_count} issues...")
        remaining = fix_category(category, args.dry_run)
        fixed = before_count - remaining
        
        if args.dry_run:
            print(f"  Would fix {fixed} issues (dry run)")
        else:
            print(f"  Fixed {fixed} issues, {remaining} remaining")
    
    if not args.dry_run:
        total_after = count_issues()
        print(f"Overall: Fixed {total_before - total_after} issues, {total_after} remaining")


if __name__ == "__main__":
    main() 