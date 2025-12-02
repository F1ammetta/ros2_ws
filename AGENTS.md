# Agent Guidelines for ROS Workspace

This document outlines the conventions and commands for agents operating within this ROS workspace.

## Build/Lint/Test Commands

*   **Build all packages:** `colcon build`
*   **Run all tests:** `colcon test`
*   **Run tests for a specific package (e.g., `lab01_pkg`):** `colcon test --packages-select lab01_pkg`
*   **Run a single Python test file (e.g., `test_flake8.py` in `lab01_pkg`):** `pytest src/lab01_pkg/test/test_flake8.py`
*   **Lint Python code (Flake8):** `flake8 src/`
*   **Lint Python docstrings (PEP 257):** `pydocstyle src/` (assuming `pydocstyle` is installed, as `test_pep257.py` implies PEP 257 checks)

## Code Style Guidelines (Python)

*   **Formatting:** Adhere to PEP 8. Use `flake8` for checking.
*   **Imports:** Organize imports according to PEP 8 (standard library, third-party, local application/library specific imports).
*   **Naming Conventions:** Follow PEP 8 for variable, function, class, and module names.
*   **Docstrings:** Use PEP 257 for docstring conventions.
*   **Error Handling:** Implement robust error handling using `try-except` blocks where appropriate.
*   **Types:** Use type hints for function arguments and return values where it improves readability and maintainability.

## Cursor/Copilot Rules

No specific Cursor or Copilot rules were found in this repository.
