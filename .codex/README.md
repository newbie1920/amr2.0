# AMR2.0 Codex Project Pack

This folder keeps Codex-specific project guidance in sync with the existing
Claude setup without changing runtime code.

## Files

- `AGENTS.md`: local entrypoint for the Codex project pack.
- `instructions.md`: project operating rules for Codex sessions.
- `memory.md`: stable AMR2.0 facts and validation commands.
- `settings.json`: small command/path map for local automation.
- `rules/`: workflow, tech defaults, and UI design rules.
- `skills/`: reusable AMR2.0 and GitNexus workflows.
- `checklists/`: task-start and validation checklists.
- `agents/researcher.md`: bounded research profile.
- `agents/reviewer.md`: review and verification profile.

## Source of Truth

- Root `AGENTS.md` remains the cross-agent instruction entrypoint.
- `.claude/` remains the Claude-specific configuration.
- Keep this folder short and practical; avoid duplicating long docs from
  `docs/`, `PROJECT_CONTEXT.md`, or `PROJECT_MEMORY.md`.

## Common Loads

- Robot upgrade: `instructions.md`, `skills/amr2-robot-upgrade.md`,
  `rules/workflow.md`.
- UI/RViz change: `rules/design.md`, `rules/tech-defaults.md`,
  `checklists/validation.md`.
- Risk or refactor check: `skills/gitnexus.md`, `rules/workflow.md`.
