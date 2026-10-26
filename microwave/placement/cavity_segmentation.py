"""SAM 3 segmentation of the open microwave's interior: the cavity MASK (raw image pixels).

SAM 3 only says WHICH PIXELS are the inside of the microwave; it knows nothing about distance.
Every 3D quantity downstream comes from the aligned depth at those pixels (`floor_target.py`).

Model: the repo's `scripts/detect_handle_sam3.Sam3HandleDetector` (same loader/weights the fridge and
the earlier placement detector use; MTerryJack/sam3, safetensors only). Prompt "inside of open
microwave": 0.91 on the 2026-10-20 rig frame (tests/data/microwave_lookin_2026-10-20.npz), the mask
covering floor, side walls, back wall and ceiling up to the opening. ("microwave floor" scored
0.60-0.69 and missed the back half of the floor; the floor is found from depth instead.)

Upright image: SAM 3 is run on the image turned by a multiple of 90 deg so world-up is image-up,
derived from `base_T_cam` (gravity in the camera frame) -- not from knowing the camera is a wrist
camera rolled on its mount. An upright fixed camera simply gets 0 turns. On the 10-20 frame the raw
(rotated) image found nothing with any prompt; upright it scored 0.91.

Filtering (each failure is a refusal with the reason, never a weaker mask):
  score >= min_score; keep the largest connected piece (drops stray islands, e.g. a reflection in the
  open door); area >= min_area_frac of the image; then grown by dilate_px so the opening's rim and the
  face above it are in the cloud too (the cavity fit takes the top from there).
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from point_cloud import upright_k

REPO_ROOT = Path(__file__).resolve().parents[2]


class SegmentationError(RuntimeError):
    """No usable cavity mask (message says why)."""


@dataclass
class SegmentationConfig:
    prompt: str = "inside of open microwave"
    repo: str = "MTerryJack/sam3"          # detect_handle_sam3.DEFAULT_REPO
    dtype: str = "bf16"
    threshold: float = 0.5                 # SAM 3's own instance threshold
    min_score: float = 0.6                 # refuse a weaker cavity
    min_area_frac: float = 0.03            # of the image
    dilate_px: int = 20
    upright: str = "gravity"               # "gravity" (turn from base_T_cam) | "none"


@dataclass
class CavityMask:
    mask: np.ndarray        # HxW bool, raw image pixels, filtered (+ dilated)
    sam_mask: np.ndarray    # HxW bool, SAM 3's best instance before filtering (for the overlay)
    score: float
    k_upright: int          # quarter turns applied before SAM 3
    prompt: str
    notes: list


class Sam3Model:
    """The real model (heavy import: torch + transformers, ~5-8 s load on the Jetson)."""

    def __init__(self, cfg: SegmentationConfig):
        sys.path.insert(0, str(REPO_ROOT / "scripts"))
        import detect_handle_sam3
        self.det = detect_handle_sam3.Sam3HandleDetector(prompt=cfg.prompt, repo=cfg.repo, dtype=cfg.dtype,
                                                         thresh=cfg.threshold)

    def segment(self, rgb):
        """(mask, score) of the best instance in an RGB image, or None."""
        r = self.det.segment(rgb)
        return None if r is None else (r[0], r[1])


def largest_component(mask):
    import cv2
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    if n <= 1:
        return np.zeros_like(mask, bool), 0
    best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return lab == best, n - 1


class CavitySegmenter:
    """frame -> CavityMask. `model` is anything with segment(rgb) -> (mask, score) | None (tests pass a fake)."""

    def __init__(self, cfg: SegmentationConfig = None, model=None):
        self.cfg = cfg or SegmentationConfig()
        if self.cfg.upright not in ("gravity", "none"):
            raise ValueError(f"segmentation.upright must be 'gravity' or 'none', got {self.cfg.upright!r}")
        self.model = model if model is not None else Sam3Model(self.cfg)

    def segment(self, frame):
        import cv2
        cfg = self.cfg
        k = upright_k(frame.base_T_cam) if cfg.upright == "gravity" else 0
        img = np.ascontiguousarray(np.rot90(frame.bgr, k)) if k else frame.bgr
        out = self.model.segment(np.ascontiguousarray(img[:, :, ::-1]))       # SAM 3 wants RGB
        if out is None:
            raise SegmentationError(f"SAM 3 found no '{cfg.prompt}' (is the door open and the cavity in view?)")
        m, score = out
        m = np.asarray(m) > 0
        if m.ndim != 2:
            raise SegmentationError(f"unexpected mask shape {m.shape}")
        raw = np.ascontiguousarray(np.rot90(m, -k)) if k else m
        if raw.shape != frame.bgr.shape[:2]:
            raise SegmentationError(f"mask {raw.shape} does not match the image {frame.bgr.shape[:2]}")
        if score < cfg.min_score:
            raise SegmentationError(f"SAM 3 cavity score {score:.2f} < {cfg.min_score} -- not trusted")
        mask, n_pieces = largest_component(raw)
        notes = []
        if n_pieces > 1:
            notes.append(f"kept the largest of {n_pieces} mask pieces "
                         f"({mask.sum()} of {raw.sum()} px)")
        frac = mask.mean()
        if frac < cfg.min_area_frac:
            raise SegmentationError(f"cavity mask covers {frac * 100:.1f}% of the image (< {cfg.min_area_frac * 100:.0f}%)"
                                    " -- too small/far to measure")
        if cfg.dilate_px > 0:
            ker = np.ones((2 * cfg.dilate_px + 1,) * 2, np.uint8)
            mask = cv2.dilate(mask.astype(np.uint8), ker) > 0
        return CavityMask(mask, raw, float(score), k, cfg.prompt, notes)
