---
name: close-sprint
description: Validates and closes a completed sprint — verifies tickets, merges branch, archives sprint
---

# Close Sprint Skill

This skill closes a completed sprint using the `close_sprint` MCP tool,
which handles the full lifecycle.

## Inputs

- Active sprint directory in `docs/clasi/sprints/NNN-slug/`
- All tickets for this sprint should be `done`

## Process

1. **Confirm with stakeholder**: Present a summary of the sprint —
   list the completed tickets and key changes. Ask whether to proceed:
   - "Close sprint and merge to main" (recommended)
   - "Review completed work first"

   If the stakeholder chooses to review, invoke the `sprint-review`
   skill first.

2. **Call close_sprint**: Invoke the `close_sprint` MCP tool:
   ```
   close_sprint(
       sprint_id="NNN",
       branch_name="sprint/NNN-slug",
       main_branch="master",
       push_tags=True,
       delete_branch=True,
       test_command="uv run pytest",  # or "" to skip tests
   )
   ```

   The `test_command` parameter controls how tests are run:
   - Omit or `None`: runs `uv run pytest` (default)
   - Custom string (e.g., `"npm test"`): runs that command
   - Empty string `""`: skips tests entirely (non-Python projects)

   The tool handles internally:
   - Pre-condition verification with self-repair
   - Run tests (if test_command is provided)
   - Archive sprint directory to `sprints/done/`
   - Update state DB, release execution lock
   - Version bump and git tag
   - Merge to master, push tags, delete branch

3. **Report result**: On success, report the version tag and merged
   branch. On error, report the blocker and recovery steps.

## Issue Preconditions

Close-sprint hard-fails if any `<sprint>/issues/<filename>` (at the top
level, not in `done/`) still has `status: in-progress`. Self-repair
handles done-tagged files automatically, but in-progress issues require
explicit resolution.

**Resolution paths:**
- **Tickets are done but issue not marked done**: this should not happen
  in the happy path. Call `move_issue_to_done` explicitly.
- **Issue has work remaining**: call `split_issue` to split the remaining
  work into a new issue, then either defer it (it stays in the pool for
  the next sprint) or call `create_ticket` to bring it into the current
  sprint before closing.
- **Issue is intentionally deferred**: set `completes_issue: false` on
  the ticket(s) referencing this issue. Close-sprint will then skip the
  hard-fail for that issue.

## Output

- Sprint branch merged to main and deleted
- Sprint document moved to `docs/clasi/sprints/done/`
- Sprint completion summary
