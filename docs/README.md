# Documentation

Start with the [project quickstart](../README.md).

- [ROS 2: launch, interfaces, resources and optional depth](ros2.md)
- [Shared web/MCP API and private transport contract](thin-api.md)
- [Deployment and private state](../deploy/OPERATIONS.txt)
- [Arm geometry and command conventions](z70-arm-model.md)
- [Manipulation](../src/z70_manipulation/README.md),
  [mapping/depth](../src/z70_mapping/README.md),
  [odometry/mosaic](../src/z70_odometry/README.md),
  [Foxglove](../src/z70_bringup/foxglove/README.md)
- Reusable checks: [shared cameras](validation/shared-camera-mode.md),
  [camera throughput](validation/camera-throughput.md),
  [passive chassis calibration](validation/chassis-odometry-trial.md)
- [Camera preview-rejection incident](validation/camera-preview-start-rejection.md)

Dated session briefs, migration records and `archive/` are historical evidence,
not current setup instructions or standing authorization for hardware access.
Calibration originals remain under `../calibration/`; installed package resources
are runtime defaults. Source and generated ROS interfaces define the implementation.
