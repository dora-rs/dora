# Automated Linting with Ruff

This document describes the approach for automated linting of the Python codebase using [Ruff](https://github.com/astral-sh/ruff), a fast Python linter written in Rust.

## Overview

The codebase has a significant number of linting issues (over 20,000) across various categories. To address these issues in a manageable way, we've implemented an iterative approach that:

1. Fixes issues by category (e.g., E for errors, F for flake8, W for warnings)
2. Automates the process of creating PRs for each category
3. Allows for controlled, incremental improvements to the codebase

## Setup

1. Make sure you have Ruff installed:
   ```bash
   pip install ruff
   ```

2. The configuration for Ruff is in `pyproject.toml`, which includes:
   - Rules selection and ignores
   - Line length configuration
   - Per-file rule exceptions
   - Other Ruff-specific settings

## Tools

### 1. `fix_linting.py`

A Python script that:
- Lists all available rule categories
- Counts issues by category
- Fixes issues for a specific category or all categories
- Supports dry-run mode to preview changes

Usage:
```bash
# List all available rule categories
./fix_linting.py --list-categories

# Fix issues for a specific category
./fix_linting.py --category E

# Preview fixes without making changes
./fix_linting.py --category F --dry-run
```

### 2. `fix_linting.sh`

A shell script that automates the process of:
- Fixing issues for a specific category
- Creating a git branch for the fixes
- Committing the changes
- Preparing for PR creation

Usage:
```bash
# Show help
./fix_linting.sh --help

# List all available rule categories
./fix_linting.sh --list-categories

# Fix issues for a specific category
./fix_linting.sh --fix-category E

# Fix issues and create a PR branch
./fix_linting.sh --create-pr F

# Preview fixes without making changes
./fix_linting.sh --dry-run --fix-category W
```

## Recommended Workflow

1. Start with the most critical categories (E, F, W) that have the most impact on code quality
2. Create separate PRs for each category to keep changes manageable
3. Run tests after each fix to ensure no functionality is broken
4. Gradually address more categories as the codebase improves

## Example PR Process

```bash
# List categories to identify which to fix first
./fix_linting.sh --list-categories

# See how many issues would be fixed for a category
./fix_linting.sh --dry-run --fix-category E

# Create a PR branch with fixes for that category
./fix_linting.sh --create-pr E

# Push the branch and create a PR
git push -u origin fix-linting-E-20231101
```

## Monitoring Progress

As PRs are merged, the overall number of linting issues will decrease. You can monitor progress with:

```bash
ruff check --select ALL . | wc -l
```

## Future Improvements

Once the most critical issues are fixed, consider:

1. Enabling more strict rules in `pyproject.toml`
2. Adding pre-commit hooks to prevent new issues
3. Integrating linting checks into CI/CD pipelines 