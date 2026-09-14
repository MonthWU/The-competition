# Challenge Two Spade-2 Offline Reconstruction

- Decision: `PASS_SPADE2_RECONSTRUCTED_WITH_INTERCHANGEABLE_ORDER`
- Jetson used: `False`
- Piece count: `4`
- Best layout slots `[top_left, top_right, bottom_left, bottom_right]`: `[4, 1, 2, 3]`
- Best 180-degree rotated slots: `[0, 1]`
- Score / margin: `6.094536` / `0.001746`
- Top/bottom vertical seam scores: `0.310152` / `0.425504`
- Opposite-corner score: `0.264773`
- Central-pip score: `0.648776`

## Outputs

- Rectified A4: `D:\CodexFolder\JetsonNano\Copyfiles\26E_vision\validation\goal_spade2_20260731_2345\rectified_a4.jpg`
- Piece sheet: `D:\CodexFolder\JetsonNano\Copyfiles\26E_vision\validation\goal_spade2_20260731_2345\piece_sheet.jpg`
- Reconstructed clean: `D:\CodexFolder\JetsonNano\Copyfiles\26E_vision\validation\goal_spade2_20260731_2345\spade2_reconstructed_clean.jpg`
- Reconstructed overlay: `D:\CodexFolder\JetsonNano\Copyfiles\26E_vision\validation\goal_spade2_20260731_2345\spade2_reconstructed_overlay.jpg`

## Boundary

This proves only that the captured Windows-side still image can be segmented and reassembled into a black spade-2 face. It is not Jetson runtime, ROS, serial, HMI, MCU, or mechanism acceptance evidence.

The best two candidates are close because one pair carries very weak distinguishing texture. The reconstructed card face is still a black spade 2; exact physical ordering of the low-texture halves should be rechecked with a sharper capture before control output.
