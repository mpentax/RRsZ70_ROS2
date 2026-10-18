# Migration history and current owners

The directory cutover is complete. This directory retains dated design, inventory
and qualification records; it is not the operator onboarding path.

## Current documentation

- [Getting started](../../README.md): install, configured-user setup and bringup.
- [ROS guide](../ros2.md): ownership, topics, actions, estimates and installed resources.
- [Command and shared web/MCP API](../thin-api.md): current authenticated routes,
  task identity, dispatcher/stop semantics and delivery uncertainty.
- [Arm model](../z70-arm-model.md), [manipulation](../../src/z70_manipulation/README.md),
  [mapping/depth](../../src/z70_mapping/README.md) and
  [odometry](../../src/z70_odometry/README.md): package-owned behavior and limits.

ROS owns execution, estimation and shared acquisition; the private vendor adapter
owns transport/authentication, and web/MCP are thin clients. Advanced RPC uses the
same dispatcher as manual controls and tasks. Runtime resources install with their
owning packages; mutable arm/chassis state uses durable application data, not
measurement archives or package shares. Offline software success is not physical
completion, calibrated accuracy, camera recovery or repeatable manipulation.

## Retained history

- [Pre-cutover runtime inventory](runtime-inventory.md),
  [asset inventory](asset-inventory.md), [layout decisions](layout-and-cutover.md)
  and [phase-1 verification](verification.md).
- [Phase 2](phase2.md): shared observation driver.
- [Phase 3](phase3.md): execution/estimation cutover and historical resource proof.
- [Phase 4](phase4.md): shared MCP/web/DDS qualification.
- [Phase 5](phase5.md): realized layout, replay and reported offline qualification.
- Original [tracked-asset snapshot](tracked-assets.json) and [path map](path-map.json)
  remain immutable inventories of their original index, not HEAD.
- [Resource/import reference](phase3-resources.md) distinguishes current source
  ownership from its retained migration receipts. The old [interface](interfaces.md),
  [command](phase3-command-api.md) and [HTTP](phase4-api.md) entrypoints redirect to
  current owners without requiring operators to traverse phases.

The former `scripts/migration-audit.test.ts` and
`scripts/phase5-preservation.test.ts` are unavailable; there is no current audit
command here. Missing historical logs/manifests are identified on the phase pages.
Do not synthesize them or regenerate the pre-cutover index from the current tree.
The preserved distribution notices, METADATA and SHA256SUMS under
`retired-environment/` are provenance/legal artifacts, not installation guidance.
