---
description: How to review a pull request locally, including testing the merged result
---

# PR Review Workflow

## Goal
Evaluate a PR's code **as it would look after merging into the target branch** (usually `main`), so you can catch runtime issues before approving.

---

## First: Does the PR branch already include `main`?

Check whether the PR author merged (or rebased on) `main` before opening the PR.

### ✅ YES — source branch already includes `main` (best practice)
Just check out the source branch directly. It already represents the post-merge result.

```bash
git fetch origin
git checkout <PR-BRANCH-NAME>   # e.g. shooter-control
```

Skip to **Step: Review and Test** below.

---

### ⚠️ NO — source branch is behind `main`
You need to simulate the merge locally using a temporary review branch.

```bash
git fetch origin

# Create a review branch from main
git checkout -b review/pr-<NUMBER> origin/main

# Merge in the PR branch to simulate the merge
git merge origin/<PR-BRANCH-NAME>
```

This gives you a local branch that reflects **exactly what the codebase would look like after the merge**.

---

## Review and Test
- Run `./gradlew build` to check for compile errors
- Deploy to the robot or simulator and test for runtime behavior
- Read the changed files in VS Code

## Making Fixes
Always make edits on the **source branch**, not the review branch:

```bash
git checkout <PR-BRANCH-NAME>
# make edits...
git add . && git commit -m "fix: <description>"
git push origin <PR-BRANCH-NAME>
```

If using a review branch, update it cheaply to re-evaluate (no re-editing):
```bash
git checkout review/pr-<NUMBER>
git merge origin/<PR-BRANCH-NAME>
```

## Clean Up (if you used a review branch)
```bash
git checkout main
git branch -d review/pr-<NUMBER>
```

---

## Key Concepts

| Term | Meaning |
|------|---------|
| **Source branch** | The branch the PR author worked on (e.g. `shooter-control`) |
| **Target branch** | The branch the PR merges INTO (usually `main`) |
| **Review branch** | Temporary local branch — only needed if source branch is behind `main` |

> **Best practice for PR authors:** Before opening a PR, merge `main` into your source branch and resolve any conflicts. This means reviewers can test your branch directly without needing a separate review branch.
