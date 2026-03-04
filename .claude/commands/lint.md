Run ruff linter and formatter on the robots_realtime package.

Steps:
1. Check for lint errors:
   ```bash
   uv run ruff check robots_realtime/
   ```

2. Auto-fix safe issues:
   ```bash
   uv run ruff check --fix robots_realtime/
   ```

3. Format code:
   ```bash
   uv run ruff format robots_realtime/
   ```

Line length is 119 chars. Config is in pyproject.toml [tool.ruff].
Do not modify pyproject.toml ruff settings unless the user asks.
Report any remaining issues that need manual fixes.
