# AGENTS.md

This document defines how agents—human and automated—operate within this codebase.  
It follows **Extreme Programming (XP)** principles and adopts **Conventional Commits** for clear traceability.

## Roles

- **Feature Agent (developer or pair):**  
  Breaks stories into smallest testable increments. Practices TDD. Keeps work in progress minimal.

- **Review Agent:**  
  Verifies Definition of Done: runnable demo, passing tests, updated docs. Requests scope reduction if PR grows.

- **Maintainer Agent:**  
  Curates backlog, merges only green builds, enforces API stability, plans refactors.

- **CI Agent (automation):**  
  Executes linting, unit/property tests, coverage, security and license scans.

- **Release Agent (automation):**  
  Generates changelog, semantic version bump, tags, and artifact publishing after successful merge.

- **Docs Agent:**  
  Keeps guides, tutorials, and API references synchronized with implementation.

## Code Generation

Aim at generating the minimal amount of code.

When generating code, consider the following questions:

- What is the purpose of this code?  
- What are the dependencies and what are the possible regressions?
- Would new user be able to easily understand the code?
- Is it overly complex?
- What are the edge cases?
- What are the possible alternatives and tradeoffs?

Before implementing changes, provide a list of affected files and code areas
and a summary of those changes as well as at least one alternative that was
considered and why it was rejected.

When expanding the API, confirm the new surface.

Confirm before applying any changes.

## Testing

- All tests reside under `test/{unit,property,fuzz,integration,conformance}/`.
- Common generators, models, and case templates live in `test/support/`.

## Definition of Done

- **Tests:** Appropriate coverage (unit/property/fuzz/conformance/integration as relevant).
- **Documentation:** README or guide reflects current design and usage.
- **Observability:** Metrics/logging hooks instrumented where applicable.
- **Debt:** All remaining TODOs are converted into tracked issues before merge.

## Conventional Commits

Format:  
`<type>(<scope>): <message>`

Common types:

- `feat:` user-visible feature  
- `fix:` bug fix  
- `refactor:` behavior-preserving change  
- `perf:` performance improvement  
- `test:` test-only change  
- `docs:` documentation change  
- `build:` CI/build system  
- `chore:` maintenance work  
- `revert:` revert of previous commit  

Add `BREAKING CHANGE:` footer when relevant.

Examples:

- `feat(sync): add digest-first handshake`
- `fix(store-sqlite): handle WAL rotation`
- `refactor(crdt): extract dvv context module`
- `docs(guide): expand replay walkthrough`

## Review Policy

- Prefer **small, frequent merges** (<300 lines diff).  
- Every change must be demonstrable (test or demo).  
- Reject complexity not required by a failing test or explicit story.  
- Keep trunk green — integration tests must always pass.
