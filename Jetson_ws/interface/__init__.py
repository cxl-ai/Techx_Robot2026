"""Core layer — interfaces, types, and exceptions for TECHX_vision."""

from interface.interfaces import ICamera, IDetector, ITracker, ISender
from interface.types import Frame, Detection, Track, Target3D
from interface.exceptions import (
    TechxVisionError,
    CameraError,
    DetectorError,
    TrackerError,
    SenderError,
    PipelineError,
    ConfigError,
)

__all__ = [
    # Interfaces
    "ICamera", "IDetector", "ITracker", "ISender",
    # Types
    "Frame", "Detection", "Track", "Target3D",
    # Exceptions
    "TechxVisionError", "CameraError", "DetectorError",
    "TrackerError", "SenderError", "PipelineError", "ConfigError",
]
