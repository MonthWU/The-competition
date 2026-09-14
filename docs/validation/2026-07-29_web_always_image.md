# Debug Web continuous image validation

Date: 2026-07-29

## Regression

The debug Web node subscribed to `puzzle/solver_image_debug`. That topic is event-driven and only publishes after the solver creates a debug plan, so the Web page showed no image in `IDLE` and `WAIT_TOOL_CLEAR`.

The camera and perception path were healthy: `puzzle/image_debug` had one publisher and returned a live 1920x1080 image while the Web node had no subscription to it.

## Fix

`web_tuner_node.image_topic` now uses `puzzle/image_debug`. The perception node publishes this stream continuously whenever debug mode enables `publish_debug_image`, independent of coordinator task state.

## Jetson validation

- `vision_bringup` build: passed.
- Windows and Jetson YAML SHA256: matched.
- `IDLE`: `image_available=true`, 1920x1080, approximately 30 FPS, frame counter increasing.
- Task 1 `WAIT_TOOL_CLEAR`: task accepted and frame counter increased from 822 to 849.
- Switched to task 2 `WAIT_TOOL_CLEAR`: task accepted and frame counter increased from 849 to 911.
- MJPEG endpoint: returned multipart `--frame`, `Content-Type: image/jpeg`, and JPEG SOI bytes.
- Five debug nodes remained online after validation.

No tool-clear signal or placement command was sent during this test.

## Backups

- Windows: `D:\CodexFolder\JetsonNano\Copyfiles\backups\26E_vision\20260729-233523-web-always-image`
- Jetson: `/home/jetson/ProjectsByMonthWU/VisionJetson/backups/26E_vision/20260729-233523-web-always-image`

Restore the backed-up `vision_system.yaml`, delete this document, selectively deploy the YAML, rebuild `vision_bringup`, and repeat the debug Web image checks to roll back.
