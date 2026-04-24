# Contributing to OpenVerse

Thanks for your interest in contributing! This document explains the process for reporting bugs, proposing features, and submitting code.

> **Note:** OpenVerse is in early development and we are not strict about following every guideline to the letter — contribute in whatever way works for you. That said, following the conventions below makes reviewing and integrating changes much easier, and it is genuinely appreciated.

For a full technical reference of the codebase (modules, data structures, rendering pipeline, physics), see [ARCHITECTURE.md](ARCHITECTURE.md).

---

## Table of Contents

1. [Code of Conduct](#1-code-of-conduct)
2. [Reporting Bugs](#2-reporting-bugs)
3. [Requesting Features](#3-requesting-features)
4. [Branching Strategy](#4-branching-strategy)
5. [Submitting a Pull Request](#5-submitting-a-pull-request)
6. [Code Review Process](#6-code-review-process)
7. [Labels](#7-labels)

---

## 1. Code of Conduct

This project follows the [Contributor Covenant Code of Conduct](CODE_OF_CONDUCT.md). By participating you agree to uphold it.

---

## 2. Reporting Bugs

Open an **Issue** and apply the `bug` label. Please include:

**Title:** Short and specific — e.g. `Saturn rings disappear when camera is inside the ring plane` not `rings broken`.

**Body:**

```
### Description
What went wrong?

### Steps to Reproduce
1. ...
2. ...

### Expected Behaviour
What should have happened?

### Actual Behaviour
What happened instead?

### Environment
- OS:
- GPU / Driver:
- Build type (Debug / Release):

### Additional Context
Screenshots, logs, or anything else relevant.
```

---

## 3. Requesting Features

Open an **Issue** and apply the `enhancement` label.

```
### Summary
One-sentence description of the feature.

### Motivation
Why is this useful? What problem does it solve?

### Proposed Approach
How might it be implemented? Reference relevant files from ARCHITECTURE.md if applicable.

### Alternatives Considered
Other approaches you thought about and why you ruled them out.
```

---

## 4. Branching Strategy

| Branch | Purpose |
|---|---|
| `main` | Always stable and buildable. Direct commits are not allowed. |
| `feat/<name>` | New features (e.g. `feat/jupiter-moons`) |
| `fix/<name>` | Bug fixes (e.g. `fix/ring-lod-flicker`) |
| `docs/<name>` | Documentation only (e.g. `docs/architecture-update`) |
| `refactor/<name>` | Refactoring without behaviour change |

**Rules:**
- Branch off `main`, merge back into `main` via PR.
- Keep branches focused — one feature or fix per branch.
- Delete the branch after the PR is merged.

---

## 5. Submitting a Pull Request

1. Make sure the project builds without warnings (`-Wall -Wextra`).
2. Test the change at multiple simulation speeds and zoom levels.
3. Open a PR against `main` with the following format:

**Title:** `[type] Short description` — e.g. `[feat] Add Galilean moons for Jupiter`

**Body:**

```
### What does this PR do?
Short summary of the change.

### Related Issue
Closes #<issue number> (if applicable)

### Changes
- file.c: what changed and why
- shader.frag: what changed and why

### Testing
How did you verify the change? What did you check for regressions?

### Screenshots / Videos
(For visual changes — before/after if possible)
```

---

## 6. Code Review Process

- Every PR needs **at least one approving review** before merge.
- The reviewer aims to respond within **7 days**. If there is no response after a week, feel free to ping in the PR comments.
- Address all review comments before requesting a re-review.
- The author merges after approval (not the reviewer).

**What reviewers check:**
- Does it build cleanly?
- Are the physics/rendering changes correct?
- Does it follow the existing code style (snake_case, `g_` globals, C99)?
- Are new bodies/data added to `assets/universe.json` rather than hardcoded?
- Does it handle the `alive = 0` body lifecycle correctly?

---

## 7. Labels

| Label | Meaning |
|---|---|
| `bug` | Something is broken |
| `enhancement` | New feature or improvement |
| `physics` | Relates to the integrator, orbital mechanics, or collision system |
| `rendering` | Relates to shaders, the render pipeline, or visual effects |
| `data` | Relates to `universe.json` or body definitions |
| `docs` | Documentation only |
| `good first issue` | Self-contained, well-scoped — good entry point for new contributors |
| `wontfix` | Out of scope or intentionally not addressed |
