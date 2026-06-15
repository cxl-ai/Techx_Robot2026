from __future__ import annotations

from typing import Iterable, List

from interface.interfaces import IDetector
from interface.types import Detection, Frame
from utils.logger import get_logger

log = get_logger(__name__)


class CompositeDetector(IDetector):
    """Run several detectors and merge their outputs into one pipeline stream.

    PipelineEngine expects one IDetector instance. CompositeDetector keeps that
    contract while allowing main.py to assemble the enabled detectors from the
    runtime configuration.
    """

    def __init__(self, detectors: Iterable[IDetector], strict_load: bool = True):
        self.detectors = list(detectors)
        self.strict_load = strict_load
        self._loaded = False

    def load(self) -> bool:
        ok_count = 0
        failed_labels = []
        for detector in self.detectors:
            label = getattr(detector, "backend_name", detector.__class__.__name__)
            try:
                if getattr(detector, "is_loaded", False) or detector.load():
                    ok_count += 1
                    continue
                failed_labels.append(str(label))
                log.error("detector load returned false: %s", label)
            except Exception as e:
                failed_labels.append(str(label))
                log.error("detector load failed: %s — %s", label, e)

        if self.strict_load:
            self._loaded = ok_count == len(self.detectors)
        else:
            self._loaded = ok_count > 0

        log.info(
            "CompositeDetector loaded %d/%d detectors (strict_load=%s)",
            ok_count,
            len(self.detectors),
            self.strict_load,
        )
        if failed_labels:
            log.error("CompositeDetector failed detectors: %s", ", ".join(failed_labels))
        return self._loaded

    def detect(self, frame: Frame) -> List[Detection]:
        output: List[Detection] = []
        for detector in self.detectors:
            try:
                output.extend(detector.detect(frame))
            except Exception as e:
                log.warning("detector failed: %s", e)
        return output

    def set_backend(self, backend: str) -> str:
        labels = []
        for detector in self.detectors:
            fn = getattr(detector, "set_backend", None)
            try:
                if callable(fn):
                    labels.append(str(fn(backend)))
                else:
                    labels.append(getattr(detector, "backend_name", detector.__class__.__name__))
            except Exception as e:
                log.warning("detector backend switch failed: %s", e)
                labels.append(f"{detector.__class__.__name__}: backend switch failed")
        return " + ".join(labels) if labels else "no detector"

    @property
    def is_loaded(self) -> bool:
        return self._loaded

    @property
    def class_names(self) -> dict:
        names = {}
        for detector in self.detectors:
            names.update(getattr(detector, "class_names", {}) or {})
        return names

    @property
    def backend_name(self) -> str:
        labels = []
        for detector in self.detectors:
            labels.append(getattr(detector, "backend_name", detector.__class__.__name__))
        return " + ".join(labels) if labels else "no detector"
