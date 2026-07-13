# AGENTS.md

## Scope
- This repository is the Phase 1 skeleton for `agt_navigation_v2`.
- Only create architecture docs, package skeletons, interface stubs, tests, and dependency manifests.
- Do not migrate third-party algorithm source code into this repository during Phase 1.

## Hard Rules
- Migrate one module or one data chain at a time.
- Do not hardcode usernames, workspace paths, device paths, or map paths.
- Do not allow multiple nodes to publish the same TF edge.
- Do not modify validated parameters or datasets from the legacy repository without explicit approval.
- Every change that affects architecture or interfaces must update `docs/`, this file, and `docs/migration/migration_matrix.md`.

## Phase 1 Allowed Work
- Create repository directories and ROS 2 package skeletons.
- Define TF, topic, message, service, and action contracts.
- Add package READMEs, launch placeholders, config placeholders, and tests.
- Add `nav_dependencies.repos`, `.gitignore`, and CI placeholders.

## Phase 1 Forbidden Work
- Copy FAST-LIVO2, ICP, NDT, Nav2 tuning, Qt project, or semantic model code here.
- Introduce heavy runtime dependencies before a module enters its migration phase.
- Rewrite the whole repository in a single change.

## Acceptance Mindset
- Prefer small, reviewable changes.
- Keep placeholders explicit so future migration tasks know what is still missing.
- For each module, document inputs, outputs, TF responsibility, and non-goals.
