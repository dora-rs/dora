#!/bin/bash
# Script to automate the process of fixing linting issues and creating PRs

set -e

# Make the script executable
chmod +x fix_linting.py

# Function to display help
show_help() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --help                 Show this help message"
    echo "  --list-categories      List all available rule categories"
    echo "  --fix-category CATEGORY Fix issues for a specific category"
    echo "  --create-pr CATEGORY   Fix issues for a category and create a PR"
    echo "  --dry-run              Show what would be fixed without making changes"
    echo ""
    echo "Examples:"
    echo "  $0 --list-categories"
    echo "  $0 --fix-category E"
    echo "  $0 --create-pr F"
    echo "  $0 --dry-run --fix-category W"
}

# Check if git is available
if ! command -v git &> /dev/null; then
    echo "Error: git is not installed or not in PATH"
    exit 1
fi

# Check if we're in a git repository
if ! git rev-parse --is-inside-work-tree &> /dev/null; then
    echo "Error: Not in a git repository"
    exit 1
fi

# Check if there are uncommitted changes
check_uncommitted_changes() {
    if ! git diff-index --quiet HEAD --; then
        echo "Error: There are uncommitted changes in the repository"
        echo "Please commit or stash your changes before running this script"
        exit 1
    fi
}

# Create a PR branch and commit changes
create_pr_branch() {
    local category=$1
    local dry_run=$2
    local branch_name="fix-linting-${category}-$(date +%Y%m%d)"
    
    echo "Creating branch: $branch_name"
    git checkout -b "$branch_name"
    
    echo "Running linting fixes for category $category"
    if [ -n "$dry_run" ]; then
        ./fix_linting.py --category "$category" --dry-run
    else
        ./fix_linting.py --category "$category"
    fi
    
    if [ -n "$dry_run" ]; then
        echo "Dry run completed. No changes were made."
        git checkout -
        git branch -D "$branch_name"
        return
    fi
    
    echo "Committing changes"
    git add .
    git commit -m "Fix linting issues for category $category"
    
    echo ""
    echo "Branch '$branch_name' created with linting fixes for category $category"
    echo "You can now push this branch and create a PR:"
    echo "  git push -u origin $branch_name"
    echo ""
}

# Parse command line arguments
if [ $# -eq 0 ]; then
    show_help
    exit 0
fi

DRY_RUN=""

while [ $# -gt 0 ]; do
    case "$1" in
        --help)
            show_help
            exit 0
            ;;
        --list-categories)
            ./fix_linting.py --list-categories
            exit 0
            ;;
        --fix-category)
            if [ -z "$2" ]; then
                echo "Error: No category specified"
                exit 1
            fi
            if [ -n "$DRY_RUN" ]; then
                ./fix_linting.py --category "$2" --dry-run
            else
                ./fix_linting.py --category "$2"
            fi
            shift 2
            ;;
        --create-pr)
            if [ -z "$2" ]; then
                echo "Error: No category specified"
                exit 1
            fi
            check_uncommitted_changes
            create_pr_branch "$2" "$DRY_RUN"
            exit 0
            ;;
        --dry-run)
            DRY_RUN="--dry-run"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

exit 0 