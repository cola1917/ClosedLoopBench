# scene-0061 offline render-quality gate and video manifest

These tools prepare local evidence. They do not run CARLA or NuRec and their
outputs are not closed-loop acceptance results.

## Render-quality request

Run the checked-in scene-0061 request against the evidence directory:

```powershell
python runners/evaluate_render_quality.py `
  --request configs/scene0061_render_quality_request.v1.json `
  --base-dir E:/code/scene0061-evidence `
  --output E:/code/scene0061-evidence/local_development/render_quality/scene0061_light_vehicle_pose_probe.render_quality.json
```

The CLI refuses to overwrite an existing report. A request contains paired
baseline/edited frames per camera and an explicit mask provenance contract.
Missing or unreliable masks make actor ROI hole ratio, actor boundary
discontinuity, edited-region change, and unchanged-background stability
unavailable. They are never estimated from an invented bbox or threshold mask.

Reported metrics include dark/invalid pixel ratios, Laplacian sharpness, PSNR,
global SSIM, temporal flicker, actor ROI holes and boundary discontinuity when a
reliable mask exists, and a paired RGB/LiDAR actor-change contract. Global SSIM
and temporal flicker are deliberately labelled with their limitations: SSIM is
a global scalar and flicker is an unregistered whole-frame difference.

The paired RGB/LiDAR contract cannot be supplied as hand-written change
booleans. `rgb_lidar_actor_change.source_report_ref` must contain `path`,
`sha256`, `size_bytes`, `schema_version`, and `status` for a
`rgb_lidar_actor_change_source_report.v1` JSON file. The evaluator reads that
file and verifies its passed status, experiment identity, target track, paired
frame ranges, every baseline/edited RGB and LiDAR payload path/hash/size, and
that its change flags agree with the ordered payload hashes. The formal ranking
gate repeats those checks, so later source-report or payload mutation fails
closed. Until that source report is captured, use `null`; it cannot grant
`perception_eligible`.

`evidence_classification` is exactly one of:

- `perception_eligible`: all required mask, image, and multimodal gates pass;
- `control_only`: imagery can support control/state analysis, but perception
  eligibility is not established;
- `quality_stress`: retained as a rendering-robustness/limitation case;
- `rejected`: a required edit or multimodal/quality gate failed.

Vehicle removal is always `quality_stress` or `rejected`. Harmonizer output
cannot improve the classification above `source_evidence_classification`; when
that source classification is missing, Harmonizer cannot establish
`perception_eligible`.

The formal40k v4 request has only one paired frame per camera, no actor mask,
and no source report satisfying the new immutable RGB/LiDAR contract.
Its temporal and actor ROI metrics are therefore unavailable by construction,
and its expected classification is `control_only`. The older independent pose
probe remains useful debug evidence but is not accepted as a formal change
contract.

## Video shot manifest

Build the current evidence-aware shot list:

```powershell
python runners/build_scene0061_video_manifest.py `
  --evidence-root E:/code/scene0061-evidence `
  --output E:/code/scene0061-evidence/local_development/scene0061_video_manifest.json
```

Validate it again after evidence is moved or captured:

```powershell
python runners/build_scene0061_video_manifest.py `
  --evidence-root E:/code/scene0061-evidence `
  --validate E:/code/scene0061-evidence/local_development/scene0061_video_manifest.json
```

The builder checks the filesystem and derives each shot's availability as
`available`, `partial`, or `missing`. The validator rejects availability
overclaims, incomplete non-remote shots, a non-stress black-hole shot, missing
capability classes, or a remote queue inconsistent with shot flags.

The twelve mandatory capability shots cover original replay, lead braking,
pedestrian crossing/yield, CARLA bbox state, the six-camera grid, LiDAR, KPI,
frame/timestamp synchronization, black-hole limitations, algorithm identity,
and baseline-vs-edit comparison. Existing pose-probe screenshots remain clearly
separate from still-missing continuous interactive footage.
