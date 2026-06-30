# One image in, one prompt in, one annotated image out.
#
# Example:
#   python prompt_tuning.py --image path/to/0__rgb.png --prompt "fridge door handle"

import argparse

import cv2
import supervision as sv

from feeding_deployment.perception.grounded_sam import GroundedSAM


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--prompt", required=True)
    parser.add_argument("--out", default="output.png")
    args = parser.parse_args()

    image = cv2.imread(args.image)
    image = cv2.flip(image, -1)  # camera is mounted upside down

    model = GroundedSAM().grounding_dino_model
    detections = model.predict_with_classes(
        image=image,
        classes=[args.prompt],
        box_threshold=0.3,
        text_threshold=0.3,
    )

    labels = [f"{args.prompt} {c:0.2f}" for _, _, c, _, _, _ in detections]
    annotated = sv.BoxAnnotator().annotate(scene=image, detections=detections, labels=labels)
    cv2.imwrite(args.out, annotated)
    print(f"{len(detections.xyxy)} boxes, confidences={[round(float(c), 3) for c in detections.confidence]} -> {args.out}")


if __name__ == "__main__":
    main()
