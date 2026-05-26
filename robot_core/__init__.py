"""MG400 AI Robotics Platform — core package.

Layered, event-driven control stack for the Dobot MG400 arm. Dependencies flow
strictly downward:

    controller / api  ->  safety  ->  state  ->  protocol  ->  transport

Only ``transport`` (socket I/O + framing) and ``transport.feedback`` (binary
status parsing) exist as of Phase 0. Higher layers are added in later phases.
"""

__all__ = ["__version__"]

__version__ = "0.0.0"
