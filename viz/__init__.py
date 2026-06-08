"""Inspection coordinate UI backend.

Consumer of robot_core; reads SafetyBounds / forward_kinematics / FeedbackFrame
and pushes JSON over WebSocket to a Three.js top-down frontend (see
``docs/PHASE2_COORDINATE_INTERFACE_DESIGN.md``). Not imported by any robot_core
layer — the layering remains transport → state → safety → … with viz as a
read-only consumer above them.
"""
