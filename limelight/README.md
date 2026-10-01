# Limelight 3A pipeline backup

Exported 2026-10-01 from the team's Limelight 3A (firmware 2026.0) over USB at `172.29.0.1`.

## What is here

| Slot | File | Type | Notes |
| --- | --- | --- | --- |
| 0 | `pipelines/0_POLLEN_DETECTOR.vpr` | Neural detector | Confidence 0.5, exposure 1788, gain 8.5. Used by `LimelightDetectorSubsystem`. |
| 1 | `pipelines/1_APRIL_TAGS.vpr` | Viewfinder | Named APRIL_TAGS but the type is viewfinder, not fiducial. Same camera settings as slot 0. |
| 2 | `pipelines/2_Ball_Viewport.vpr` | Viewfinder | Exposure 1004, gain 9. |
| 3 | `pipelines/3_Yellow_Ball.vpr` | Color | Hue 8–35, sat min 150, val min 80, exposure 3300, gain 20, LEDs off. |
| 4 | `pipelines/4_Pipeline_Name.vpr` | Fiducial | Unnamed. Exposure 600, gain 9. |
| 5–9 | `pipelines/5_Pipeline_Name.vpr` … `9_Pipeline_Name.vpr` | Fiducial | Unused placeholders, identical to each other. |

- `fieldmap.fmap` is the field map loaded on the camera (FTC, AprilTags 20 and 24 only). Every slot reported the same map.
- `python/1_APRIL_TAGS.py` is the Python script stored in slot 1. It is a desktop webcam script (no `runPipeline`), and slot 1 is not a Python pipeline, so the camera does not run it.
- `python/stock_template.py` is the script stored in every other slot. It is Limelight's unmodified starter template.

The `.vpr` files are byte-for-byte what the camera returns from
`GET http://172.29.0.1:5807/pipeline-atindex?index=N`, which is the same content the web UI's
pipeline download button produces. They are left as single-line JSON so they stay exact.

## What is NOT here

**The neural network model and labels file for slot 0 (POLLEN_DETECTOR) are not backed up.**
The camera has no way to read them back: the REST API only accepts uploads (`/upload-nn`), and the
web UI's "Download Pipeline Pack" button fails on this camera with
`Failed to create pack: Failed to create zip archive` for every slot. Restoring
`0_POLLEN_DETECTOR.vpr` onto a blank camera gives the settings but no working detector until the
original `.tflite` and labels file are uploaded again. Whoever trained the model should add those
two files to this folder.

Also not included: the 267 snapshots stored on the camera, and camera calibration (the camera has
only the factory calibration; no custom one is stored).

## Restoring

In the web UI (`http://172.29.0.1:5801` over USB), select the slot, then use the upload button next
to the pipeline name and pick the `.vpr` file. The field map (`.fmap`) and Python script have their
own upload controls in the pipeline settings.

## Re-exporting

```sh
for i in 0 1 2 3 4 5 6 7 8 9; do
  curl -s "http://172.29.0.1:5807/pipeline-atindex?index=$i" -o "slot$i.vpr"
done
```

The field map and Python scripts are only readable for the active slot. They were read over the
web UI's websocket (port 5805, `request_fieldmap` and `request_current_python_script_for_download`)
after switching to each slot in turn.
