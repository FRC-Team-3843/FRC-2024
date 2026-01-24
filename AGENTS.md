# FRC-2024 Repository - Codex Configuration

## How to Use This Configuration

This file is for Codex-specific behavior when working in the FRC-2024 repository.

**For technical standards:** Read `STANDARDS.md` in this directory.

---

## Technical Standards

**READ THIS FIRST:** `FRC-2024\STANDARDS.md`

That file contains technical rules for FRC-2024:
- Branch guide (main = fully refactored, 2024_Archive = deprecated)
- Command-based architecture with IO-layer abstraction
- Build commands
- Copying guidelines (verify API compatibility before porting)

---

## Codex-Specific Behavior

### When Working in FRC-2024

1. **Before starting work:**
   - Check `.agent-log\changelog.md` for recent activity
   - Read `STANDARDS.md` for all technical rules and branch status
   - Verify which branch you're on (main is recommended)

2. **During work:**
   - Follow all standards in STANDARDS.md
   - Main branch: Fully refactored with command-based architecture and IO-layer
   - Do not copy code to newer projects without verifying API compatibility
   - Reference the IO-layer abstraction pattern when making architectural decisions

3. **After completing work:**
   - Log changes to `.agent-log\changelog.md`
   - Use this format:
     ```
     ### [YYYY-MM-DD HH:MM] CODEX [ACTION_TYPE]
     - Description of changes
     - Files: <paths from repo root>
     - Notes: Important context for other agents
     - PENDING: (optional) What needs follow-up
     ```

### Codex Workflow Tips

- **Branch Awareness:** Always verify which branch you're on before making changes
- **Architecture Reference:** Main branch is a reference implementation of modern WPILib patterns
- **IO-Layer Pattern:** Review how subsystems use IO interfaces for hardware abstraction
- **Testing:** Run `./gradlew build` and `./gradlew test` from project directory

---

## Cross-Agent Protocol

### Activity Logging

**Location:** `.agent-log\changelog.md`

**Before work:** Check changelog for recent changes by other agents (Claude, Gemini).
**After work:** Log all significant changes with `[CODEX]` tag.

### Handoff Tracking

**Location:** `.agent-log\handoffs.md`

If you leave work incomplete or encounter blockers:
1. Update handoffs.md with task status
2. Note what was completed and what's pending
3. Document any blockers or issues
4. Suggest which agent should continue (or mark as `ANY`)

---

## Repository Structure

```
FRC-2024/
├── STANDARDS.md               ← READ THIS for technical rules and branch guide
├── AGENTS.md (this file)      ← Codex behavior for repo
├── .agent-log/
│   ├── changelog.md           ← All activity for this repo
│   └── handoffs.md            ← Task handoffs
│
└── 2024Robot/                 ← Robot project
    ├── src/main/java/...      ← Source code
    │   ├── Robot.java
    │   ├── RobotContainer.java
    │   ├── Constants.java
    │   ├── subsystems/        ← Subsystems with IO-layer
    │   └── commands/          ← Commands
    ├── src/test/java/...      ← Tests
    ├── src/main/deploy/       ← Deploy-time assets
    └── vendordeps/            ← Vendor dependencies
```

---

## Build Commands

Run from `2024Robot/`:
```bash
./gradlew build          # Compile and package
./gradlew test           # Run JUnit 5 tests
./gradlew deploy         # Deploy to RoboRIO (team 3843)
./gradlew simulateJava   # Run simulation GUI
./gradlew clean          # Clear build outputs
```

---

## Key Reminders

- **Check branch first** - main is fully refactored, 2024_Archive is deprecated
- **Read STANDARDS.md** for architecture patterns and IO-layer details
- **APIs change season to season** - verify compatibility before copying to newer projects
- **IO-layer abstraction** - Good reference for hardware testing and simulation
- **Log all significant changes** to help other agents coordinate
- **Check changelog regularly** to avoid duplicate work

---

For cross-agent coordination protocol, see: `C:\github\AGENTS.md`
