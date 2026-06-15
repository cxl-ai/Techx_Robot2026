from __future__ import annotations

from dataclasses import dataclass


@dataclass
class TuningParams:
    conf_threshold: float = 0.45
    iou_threshold: float = 0.45
    img_size: int = 640
    min_box_area: int = 500
    max_box_area: int = 250000
    max_aspect_ratio: float = 10.0
    edge_low_confidence: float = 0.30
    edge_margin_x: int = 32
    edge_margin_y: int = 24
    nms_iou_threshold: float = 0.35
    containment_threshold: float = 0.65
    cross_class_nms_iou_threshold: float = 0.35
    cross_class_near_iou_threshold: float = 0.20
    cross_class_center_px: float = 24.0
    fake_kfs_prefer_margin: float = 0.12
    max_detections_per_frame: int = 10
    output_min_confidence: float = 0.55
    output_require_valid_depth: bool = True
    output_max_targets_per_frame: int = 6
    output_sort_by_confidence: bool = True
    output_log_interval_sec: float = 2.0
    track_iou_threshold: float = 0.1
    track_center_dist: float = 150.0
    track_confirm_frames: int = 5
    track_max_stale: int = 30
    depth_min_mm: float = 200.0
    depth_max_mm: float = 8000.0
    depth_offset_mm: float = -50.0
    depth_scale_corr: float = 1.0
    depth_roi_expansions: tuple = (0.3, 0.5, 0.7)
    depth_mad_threshold_mm: float = 300.0
    depth_roi_min_pixels: int = 5
    depth_fallback_percentile: int = 5
    depth_smooth_alpha: float = 0.6
    depth_decay_rate: float = 0.85
    depth_smooth_max_keys: int = 500
    color_min_pixels: int = 50
    color_ratio_threshold: float = 1.5
    color_hist_size: int = 5
    display_every_n: int = 8
    heatmap_alpha: float = 0.4
    roi_overlay_alpha: float = 0.2
    infer_queue_maxsize: int = 1
    camera_queue_maxsize: int = 3
    stat_window: int = 30

    # Field safety gate for graspable targets only. Unsafe targets remain visible
    # through class/conf/u/v so GMK can center on them, but their 3D XYZ is cleared
    # so GRASP requests with require_control_xyz=true cannot select them.
    quality_gate_enabled: bool = True
    quality_gate_classes: tuple = (0, 1, 2, 3, 4, 5, 100, 101, 102)
    quality_edge_margin_px: int = 30
    quality_center_error_norm: float = 0.18
    quality_min_depth_valid_ratio: float = 0.50
    quality_max_depth_spread_m: float = 0.015
    quality_require_no_fallback: bool = True
    quality_bad_confidence: float = 0.01

    def __post_init__(self):
        self.conf_threshold = max(0.0, min(1.0, self.conf_threshold))
        self.iou_threshold = max(0.0, min(1.0, self.iou_threshold))
        self.nms_iou_threshold = max(0.0, min(1.0, self.nms_iou_threshold))
        self.cross_class_nms_iou_threshold = max(0.0, min(1.0, self.cross_class_nms_iou_threshold))
        self.cross_class_near_iou_threshold = max(0.0, min(1.0, self.cross_class_near_iou_threshold))
        self.output_min_confidence = max(0.0, min(1.0, self.output_min_confidence))
        self.output_max_targets_per_frame = int(self.output_max_targets_per_frame)
        self.output_log_interval_sec = max(0.0, float(self.output_log_interval_sec))
        self.depth_smooth_alpha = max(0.0, min(1.0, self.depth_smooth_alpha))
        self.depth_decay_rate = max(0.0, min(1.0, self.depth_decay_rate))
        self.track_confirm_frames = max(1, int(self.track_confirm_frames))
        self.max_detections_per_frame = int(self.max_detections_per_frame)
        self.max_box_area = int(self.max_box_area)
        self.edge_low_confidence = max(0.0, min(1.0, float(self.edge_low_confidence)))
        self.containment_threshold = max(0.0, min(1.0, float(self.containment_threshold)))
        self.fake_kfs_prefer_margin = max(0.0, min(1.0, float(self.fake_kfs_prefer_margin)))
        self.img_size = max(32, min(1280, self.img_size))
        if self.img_size % 32 != 0:
            self.img_size = (self.img_size // 32) * 32
        self.quality_edge_margin_px = max(0, int(self.quality_edge_margin_px))
        self.quality_center_error_norm = max(0.0, min(1.0, float(self.quality_center_error_norm)))
        self.quality_min_depth_valid_ratio = max(0.0, min(1.0, float(self.quality_min_depth_valid_ratio)))
        self.quality_max_depth_spread_m = max(0.0, float(self.quality_max_depth_spread_m))
        self.quality_bad_confidence = max(0.0, min(1.0, float(self.quality_bad_confidence)))


DEFAULT_TUNING = TuningParams()
