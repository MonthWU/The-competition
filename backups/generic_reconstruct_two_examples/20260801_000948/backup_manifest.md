# Backup Manifest - generic_reconstruct_two_examples

- Created: 20260801_000948
- Scope: iterate generic reconstruction on two user-supplied examples; Windows offline only.

## Existing file backups

- tools\challenge_two_generic_reconstruct.py
  - SHA256 before: 662FBD23D6EA02F29FDCFE276F331B44569CB8083717FC1D7499FDD46573DBD9
- docs\challenge_task_two_generic_reconstruct.md
  - SHA256 before: 603FAA6F3771731CAF4634A727AA75DC5A123241D2AF852196BA55A94F66BB55

## Planned new files

- test_data/challenge_two_examples/20260801_image1.jpg
- test_data/challenge_two_examples/20260801_image2.jpg
- validation/generic_reconstruct_20260801_*/... generated outputs

## Rollback

Restore the backed-up existing files from this directory and delete the planned new files/directories.
