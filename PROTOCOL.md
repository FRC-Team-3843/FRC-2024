# FRC-2024 - Agent Coordination Protocol

**Root coordination protocol:** `C:\GitHub\PROTOCOL.md`
**Technical standards:** `STANDARDS.md` (in this directory)
**Project context:** `.agent-context.md` (in this directory)

---

## How to Use This Configuration

This file defines the **shared agent workflow** for the FRC-2024 repository. All agents (Claude, Gemini, Codex) follow the same protocol.

- **Technical rules:** Read `STANDARDS.md` for architecture, APIs, branch guide, and build commands.
- **Project state:** Read `.agent-context.md` for current phase, goals, and decisions.
- **Cross-repo protocol:** Read `C:\GitHub\PROTOCOL.md` for system-wide coordination rules.

---

## Before Starting Work

1. **Read `.agent-log\changelog.md`** by direct path -- review recent changes by all agents.
2. **Read `.agent-log\handoffs.md`** by direct path -- check for pending tasks or blockers.
3. **Read `.agent-context.md`** by direct path -- review current project state.
4. **Read `STANDARDS.md`** -- know the technical rules before touching code.

> Always use direct file Read by absolute path. On Windows, Glob can silently miss files in `.agent-log\` directories.

## During Work

- Follow all standards in `STANDARDS.md` strictly.
- Check for duplicate work before implementing new features.
- Maintain consistency with existing code patterns.

## After Completing Work

1. **Log to `.agent-log\changelog.md`** (append-only):
   ```
   ### [YYYY-MM-DD HH:MM] AGENT_NAME [ACTION_TYPE]
   - Description of changes
   - Repo: FRC-2024
   - Files modified: <paths from repo root>
   - Notes: Important context for other agents
   - PENDING: (optional) What needs follow-up
   ```

2. **Action types:** `[IMPLEMENT]`, `[REFACTOR]`, `[FIX]`, `[TEST]`, `[CONFIG]`, `[DOCS]`, `[REVIEW]`

3. **If handing off incomplete work:** Update `.agent-log\handoffs.md` with status, blockers, and next steps.

4. **Never overwrite log files.** Always Read first, then append.

---

## Repository Structure

```
FRC-2024/
  PROTOCOL.md              <- Agent workflow (this file)
  STANDARDS.md             <- Technical rules, branch guide, build commands
  .agent-context.md        <- Project state (phase, goals, decisions)
  CLAUDE.md                <- Thin pointer (Claude tag)
  GEMINI.md                <- Thin pointer (Gemini tag)
  AGENTS.md                <- Thin pointer (Codex tag)
  .agent-log/
    changelog.md           <- Activity log (append-only)
    handoffs.md            <- Work in progress / handoffs

  2024Robot/               <- Robot project
    CLAUDE.md              <- Project-level redirect
    GEMINI.md              <- Project-level redirect
    AGENTS.md              <- Project-level redirect
    src/main/java/...      <- Source code
    src/test/java/...      <- Tests
```

---

## Key Reminders

- **Always read STANDARDS.md** before making code changes.
- **This is an archive repo** -- see `.agent-context.md` for current status.
- **Log all significant changes** to help other agents coordinate.
- **Check changelog regularly** to avoid duplicate work.
- **Never overwrite** `changelog.md` or `handoffs.md`.

---

For the full cross-agent coordination protocol, see: `C:\GitHub\PROTOCOL.md`
