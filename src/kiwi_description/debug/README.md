# Generated exporter diagnostics

Everything in this directory is generated diagnostic output from the
Fusion-to-URDF/Xacro export workflow. It is retained for traceability and
debugging; it is not hand-maintained configuration and must not be used as
proof that the physical robot or navigation stack has been validated.

- `export_log.md`, `extraction_report.md`, `validation.md`: generated reports.
- `frame_model.json`, `fusion_transforms.json`, `snapshot.json`: generated
  exporter snapshots/intermediate data.

Regeneration may replace these files wholesale. Record the exporter version
and source CAD revision when updating them. Hand-maintained ROS configuration
lives in `../kiwi_drive_description/`.
