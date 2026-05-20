# Skill: GitNexus Safety Loop

Use this skill for code understanding, debugging, impact analysis, refactoring,
or pre-commit scope checks in AMR2.0.

## Quick Workflow

1. Read `gitnexus://repo/amr2.0/context` when available.
2. If the index is stale, run `npx.cmd gitnexus analyze --skip-git`.
3. For unfamiliar features, query by concept before opening many files.
4. Before editing a function/class/method, run upstream impact analysis.
5. Before commit, run change detection and compare affected scope with the
   intended task.

## Task Mapping

| Task | Use |
| --- | --- |
| Architecture or "how does X work?" | GitNexus query/context |
| "What breaks if I change X?" | Upstream impact analysis |
| Bug tracing | Query execution flows, then inspect exact files |
| Rename/refactor | GitNexus-aware rename when available |
| Pre-commit check | GitNexus detect changes |

## Risk Rules

- LOW: small symbol count, few direct callers, narrow process scope.
- MEDIUM: several callers or multiple execution flows.
- HIGH: many affected symbols, cross-subsystem impact, or real-robot behavior.
- CRITICAL: safety-critical runtime behavior without a benchmark or test gate.

Warn the user before editing when risk is HIGH or CRITICAL.
