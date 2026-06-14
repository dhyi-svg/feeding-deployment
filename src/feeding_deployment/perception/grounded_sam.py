import torch
import supervision as sv
from groundingdino.util.inference import Model
from segment_anything import sam_model_registry, SamPredictor

PATH_TO_GROUNDED_SAM = '/home/isacc/Grounded-Segment-Anything'


class GroundedSAM:
    """Shared wrapper for GroundingDINO and SAM models to avoid duplicate GPU allocation."""

    def __init__(self):
        self.DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # GroundingDINO config and checkpoint
        self.GROUNDING_DINO_CONFIG_PATH = PATH_TO_GROUNDED_SAM + "/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"
        self.GROUNDING_DINO_CHECKPOINT_PATH = PATH_TO_GROUNDED_SAM + "/groundingdino_swint_ogc.pth"

        print("Initializing shared Grounding DINO")
        self.grounding_dino_model = Model(
            model_config_path=self.GROUNDING_DINO_CONFIG_PATH,
            model_checkpoint_path=self.GROUNDING_DINO_CHECKPOINT_PATH
        )

        # SAM checkpoint
        SAM_ENCODER_VERSION = "vit_h"
        SAM_CHECKPOINT_PATH = PATH_TO_GROUNDED_SAM + "/sam_vit_h_4b8939.pth"

        print("Initializing shared SAM")
        sam = sam_model_registry[SAM_ENCODER_VERSION](checkpoint=SAM_CHECKPOINT_PATH)
        sam.to(device=self.DEVICE)
        self.sam_predictor = SamPredictor(sam)

        print("Shared GroundedSAM models loaded successfully")
