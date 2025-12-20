"""
Test runner script for the AI/Spec-Driven Book with Embedded RAG Chatbot
"""

import os
import sys
import subprocess
import argparse
from pathlib import Path

def run_tests(test_type="all", verbose=False):
    """
    Run tests for the application.

    Args:
        test_type: Type of tests to run ('unit', 'integration', 'end-to-end', 'all')
        verbose: Whether to run tests in verbose mode
    """
    print(f"Running {test_type} tests...")

    # Determine which tests to run based on type
    if test_type == "unit":
        test_path = "tests/unit"
    elif test_type == "integration":
        test_path = "tests/integration"
    elif test_type == "end-to-end":
        test_path = "tests/test_end_to_end.py"
    elif test_type == "all":
        test_path = "tests/"
    else:
        print(f"Unknown test type: {test_type}")
        return 1

    # Build pytest command
    cmd = ["pytest", test_path]
    if verbose:
        cmd.append("-v")
    else:
        cmd.extend(["-v", "--tb=short"])  # More concise output by default

    # Add coverage if requested
    if test_type in ["all", "end-to-end"]:  # Only for comprehensive runs
        cmd.extend(["--cov=src", "--cov-report=html", "--cov-report=term"])

    print(f"Running command: {' '.join(cmd)}")

    try:
        result = subprocess.run(cmd, check=True)
        print(f"Tests completed successfully!")
        return result.returncode
    except subprocess.CalledProcessError as e:
        print(f"Tests failed with return code {e.returncode}")
        return e.returncode
    except FileNotFoundError:
        print("pytest not found. Please install pytest: pip install pytest pytest-cov")
        return 1

def run_linting():
    """Run code linting and formatting checks."""
    print("Running linting checks...")

    checks = [
        ["python", "-m", "black", "--check", "src/", "tests/"],
        ["python", "-m", "flake8", "src/", "tests/"],
        ["python", "-m", "mypy", "src/"]
    ]

    for cmd in checks:
        try:
            print(f"Running: {' '.join(cmd)}")
            subprocess.run(cmd, check=True)
        except subprocess.CalledProcessError as e:
            print(f"Linting check failed: {' '.join(cmd)}")
            return e.returncode
        except FileNotFoundError:
            print(f"Tool not found for command: {' '.join(cmd)}, skipping...")
            continue

    print("All linting checks passed!")
    return 0

def run_security_scan():
    """Run basic security scanning."""
    print("Running security scan...")

    try:
        # Check for common security issues with bandit
        result = subprocess.run([
            "python", "-m", "bandit",
            "-r", "src/",
            "-c", "bandit.yaml"  # Use config file if it exists
        ], check=False)  # Don't raise exception on failure since this is just a scan

        print("Security scan completed.")
        return result.returncode
    except FileNotFoundError:
        print("Bandit not found. Install with: pip install bandit")
        print("Skipping security scan...")
        return 0

def main():
    parser = argparse.ArgumentParser(description="Test runner for AI/Spec-Driven Book with Embedded RAG Chatbot")
    parser.add_argument(
        "test_type",
        nargs="?",
        default="all",
        choices=["unit", "integration", "end-to-end", "all"],
        help="Type of tests to run (default: all)"
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Run tests in verbose mode"
    )
    parser.add_argument(
        "--lint-only",
        action="store_true",
        help="Run only linting checks"
    )
    parser.add_argument(
        "--security-only",
        action="store_true",
        help="Run only security scan"
    )
    parser.add_argument(
        "--all-checks",
        action="store_true",
        help="Run all checks: tests, linting, and security"
    )

    args = parser.parse_args()

    # Change to project root directory
    project_root = Path(__file__).parent
    os.chdir(project_root)

    exit_code = 0

    if args.lint_only:
        exit_code = run_linting()
    elif args.security_only:
        exit_code = run_security_scan()
    elif args.all_checks:
        # Run all checks in sequence
        exit_code |= run_linting()
        exit_code |= run_security_scan()
        exit_code |= run_tests(args.test_type, args.verbose)
    else:
        # Run just the specified tests
        exit_code = run_tests(args.test_type, args.verbose)

    print(f"\nTest run completed with exit code: {exit_code}")
    return exit_code

if __name__ == "__main__":
    sys.exit(main())