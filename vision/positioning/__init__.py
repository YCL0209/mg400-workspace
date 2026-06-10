"""Image-to-base-coordinate transforms (V6).

Will contain ``transform.py`` (pixel + depth_hint + intrinsics + hand_eye
-> base coordinate) and ``target.py`` (target pose generation including
z_plane / TCP offset / safety margin checks).

Depends on V2 calibration loaders + ``robot_core/kinematics/transform.py``
(the 4x4 utility, segment 2.1).
"""
