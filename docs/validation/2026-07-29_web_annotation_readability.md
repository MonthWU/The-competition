# Web annotation readability validation

Date: 2026-07-29

## Problem and root cause

The 1920x1080 perception debug image was overridden by `competition_tuning.yaml` to
a 480-pixel-wide MJPEG preview. The browser then enlarged that already-downsampled
image into an approximately 800-pixel-wide area beside a fixed status column. Small
OpenCV labels therefore lost source pixels before reaching the browser.

## Change

- Kept the OpenCV annotation font geometry unchanged.
- Increased the debug MJPEG preview to 1600 pixels wide.
- Limited the configured Web preview rate to 20 FPS to bound JPEG CPU and LAN load.
- Changed the desktop layout from 2:1 to a video-first 4:1 layout with a 1760-pixel
  maximum shell width and responsive single-column fallback.
- Added browser-rendered task, scene and piece/solve status chips over the lower edge
  of the preview so critical values do not depend on raster annotation size.
- Added fit, 1:1 pixel and fullscreen viewing modes; double-click also enters fullscreen.
- Made long scene status values span the full status-column width.

## Verification

- Windows Python AST parse: PASS.
- Served JavaScript syntax check with Node: PASS.
- HTML ID uniqueness and expected fullscreen/native controls: PASS.
- Jetson build: `web_tuner_node` and `vision_bringup` PASS.
- Windows and Jetson SHA256 values match for all three deployed files.
- MJPEG frame: 1600x900, image available continuously.
- Source camera rate during final check: 29.99 FPS.
- Configured Web preview limit: 20 FPS; observed encoded preview rate over a 3-second
  sample with an active client: 15.33 FPS. The source camera rate remained near 30 FPS.
- Task isolation regression check before the final restart: task 1 -> task 2 -> task 1
  all reached `WAIT_TOOL_CLEAR`, with generations 1 -> 2 -> 3, while image messages
  continued increasing.
- Final state after restart: task 1, `WAIT_TOOL_CLEAR`, generation 1.
- Visual checks at 1600x900 and 1280x720: PASS; annotation and Web-native status data
  were readable without changing the OpenCV font scale.
- Final debug process group/session: `41944`.
- Web: `http://192.168.137.161:5000/`.
- Log: `/home/jetson/ProjectsByMonthWU/VisionJetson/runtime_logs/26E_vision/debug-20260729-235900-web-readability.log`.

## Remaining boundary

The current camera scene does not contain a valid green A4/piece arrangement, so the
valid-scene per-piece label placement and overlap behavior were not physically tested.
The display pipeline itself preserves the same higher pixel density for those labels,
and 1:1/fullscreen inspection is available when a valid scene is placed under the camera.

## Rollback

- Windows backup:
  `D:\CodexFolder\JetsonNano\Copyfiles\backups\26E_vision\20260729-234814-web-annotation-readability`
- Jetson backup:
  `/home/jetson/ProjectsByMonthWU/VisionJetson/backups/26E_vision/20260729-234814-web-annotation-readability`
