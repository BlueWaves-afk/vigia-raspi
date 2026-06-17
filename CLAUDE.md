<<<<<<< HEAD
# Core Routing Rules

## Commands
- Build Project: `make` or `g++ -Wall -Wextra src/main.cpp -o build/main.o`
- Run Local Tests: `make test`

## Dynamic Context Routing
- Do not run blind grep or multi-file text searches.
- Use `codebase-memory-mcp` tools to trace functional architecture, dependencies, and call trees before editing.
- Localized developer guidelines are path-scoped. Review relevant notes in `.claude/rules/` automatically upon entering matching directories.
=======
# Core System Rules for `vigia-raspi`

## Architectural Constraints
- Performance: Write highly optimized, low-overhead native logic for edge computation.
- Stability: Implement aggressive sanity checks against buffer overflows, CPU load spikes, and data loss.
- Error Recovery: Always handle unexpected hardware disconnects or serial/vibration noise gracefully.

## Context Mapping & Token Efficiency
- Do not execute massive directory scans or blindly dump whole source files.
- Before making structural changes, reference the active index file at `.wiki/00_Index.md`.
- Follow internal double-bracket links `[[Note Name]]` to traverse design requirements precisely.

## Specialized Execution
- If refactoring or editing serialization protocols or low-level telemetry systems, you MUST load and satisfy the specialized Skill handbook at `.claude/skills/telemetry-validator/SKILL.md`.
- Never ask the user to verify a fix until local build compilation and internal testing gates pass completely.
>>>>>>> 459cd1e (ros2 node contracts)
