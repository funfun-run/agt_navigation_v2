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

## Vehicle Geometry Contract
- `profiles/platforms/<platform>.yaml` is the canonical source for vehicle geometry.
- Nav2, perception, and future coverage validation must consume or be contract-tested against the selected platform profile.
- Do not maintain a separate coverage-planning footprint or silently add a second safety margin.
- Changes to verified vehicle dimensions require explicit approval and corresponding contract-test updates.

## Semantic Map Contract
- Semantic geometry uses `frame_id: map`, metric coordinates, and ROS right-handed axes.
- The versioned contract is `docs/interfaces/semantic_map_schema.md` plus `src/agt_ui_bridge/config/semantic_schema.yaml`.
- Keep semantic GeoJSON separate from the base OccupancyGrid; never write semantic zones into the source PGM.
- Versioned examples live under `docs/interfaces/examples/`; runtime semantic files live under `runtime/maps/<map_id>/semantic/` and are not committed.
- Schema contract completion does not imply that coverage path validation, path repair, or coverage execution is implemented.
- Project-owned semantic data classes and file I/O under `agt_ui_bridge` are data-contract code, not third-party semantic algorithm migration.
- Keep map transforms and semantic file logic free of Qt and ROS dependencies so they remain independently testable.
- The semantic editor treats the base PGM/YAML as read-only and writes only versioned GeoJSON plus `coverage.yaml`.
- Keep the project-owned semantic editor separate from `third_party/ros_qt5_gui_app`; do not patch vendor UI code for semantic authoring.
- Semantic topology and containment checks must use Shapely/GEOS; do not replace them with project-owned polygon Boolean algorithms.
- Footprint feasibility consumes `navigation_footprint` from the selected platform profile. Any extra boundary clearance must be explicit and defaults to zero.
- The semantic map server uses standard ROS messages/services and transactional candidate loading; a failed load must not replace or clear the last valid products.
- TASK-06 keepout masks rasterize enabled exclusion/keepout zones and configurable field exterior without modifying the base map.
- Keepout masks must preserve the base OccupancyGrid metadata exactly.
- TASK-07 connects only `/agt/map/keepout_mask` to the global Nav2 costmap through a type-0 FilterInfo server.
- Keep global costmap ordering as `StaticLayer -> KeepoutFilter -> InflationLayer`; do not add the semantic mask to the local obstacle chain.
- Keepout costs are reversible filter state. Never write them into `/agt/map/global_occupancy` or the source PGM.
- Humble KeepoutFilter is fail-open before FilterInfo/mask arrives; motion procedures must verify semantic status `LOADED` instead of treating node liveness as readiness.

## Coverage Dependency Contract
- ROS 2 Humble coverage dependencies are locked by full commit SHA in `nav_dependencies.repos`.
- `opennav_coverage` uses the `humble-v2` line and Fields2Cover uses `v2.0.0`; do not mix the ordinary Humble/F2C 1.2.1 line with this contract.
- Keep coverage algorithm sources in an external vcs workspace. Do not vendor them into `agt_coverage_planning`.
- Build Fields2Cover with `USE_ORTOOLS_VENDOR=ON` and the pinned `FETCHCONTENT_SOURCE_DIR_*` inputs documented in `docs/development/coverage_dependencies.md`; no build-time network fallback or hidden old-workspace overlay is allowed.
- TASK-09 depends on Coverage Server and `ComputeCoveragePath`, but not on Coverage Navigator, BT plugins, or demos.
- Coverage requests must pass complete semantic validation and canonical platform-profile snapshot checks before changing server parameters or sending a goal.
- Humble annotated-row requests use process-private generated GML because Row Coverage Server has no in-message row input; never write generated GML into semantic source files.
- `/agt/coverage/path_raw` is not executable until TASK-10 validates costmap footprint collisions, interpolation and curvature.

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
