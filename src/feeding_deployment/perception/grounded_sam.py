import os

import groundingdino
import supervision as sv
import torch
from groundingdino.util.inference import Model
from segment_anything import SamPredictor, sam_model_registry

PATH_TO_GROUNDED_SAM = os.path.expanduser("~/Grounded-Segment-Anything")


class GroundedSAM:
    """Shared wrapper for GroundingDINO and SAM models to avoid duplicate GPU
    allocation."""

    def __init__(self):
        # torch.cuda.is_available() can report True while actual CUDA init still
        # fails (e.g. this pip torch build wants a newer CUDA driver than what's
        # installed) -- probe with a real init instead of trusting is_available().
        try:
            cuda_ok = torch.cuda.is_available() and torch.cuda.init() is None
        except RuntimeError:
            cuda_ok = False
        self.DEVICE = torch.device("cuda" if cuda_ok else "cpu")

        # GroundingDINO config (from the installed groundingdino-py package, not a
        # cloned repo) and checkpoint (Swin-B).
        self.GROUNDING_DINO_CONFIG_PATH = os.path.join(
            os.path.dirname(groundingdino.__file__),
            "config",
            "GroundingDINO_SwinB_cfg.py",
        )
        self.GROUNDING_DINO_CHECKPOINT_PATH = (
            PATH_TO_GROUNDED_SAM + "/groundingdino_swinb_cogcoor.pth"
        )

        print("Initializing shared Grounding DINO (Swin-B)")
        self.grounding_dino_model = Model(
            model_config_path=self.GROUNDING_DINO_CONFIG_PATH,
            model_checkpoint_path=self.GROUNDING_DINO_CHECKPOINT_PATH,
            device=str(self.DEVICE),
        )

        # SAM is loaded lazily on first access to `sam_predictor` (see the
        # property below). Only the FLAIR food path actually segments with SAM;
        # the appliance/handle path uses GroundingDINO boxes only. Deferring the
        # ViT-H allocation lets memory-constrained boxes (e.g. an 8GB Jetson)
        # run the appliance tasks without ever loading SAM.
        self.SAM_ENCODER_VERSION = "vit_h"
        self.SAM_CHECKPOINT_PATH = PATH_TO_GROUNDED_SAM + "/sam_vit_h_4b8939.pth"
        self._sam_predictor = None

        print("Shared GroundedSAM models loaded successfully")

    @property
    def sam_predictor(self):
        """Return the shared SAM predictor, loading ViT-H on first use."""
        if self._sam_predictor is None:
            print("Initializing shared SAM (lazy load)")
            sam = sam_model_registry[self.SAM_ENCODER_VERSION](
                checkpoint=self.SAM_CHECKPOINT_PATH
            )
            sam.to(device=self.DEVICE)
            self._sam_predictor = SamPredictor(sam)
        return self._sam_predictor
