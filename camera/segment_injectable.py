"""
Starter: Segment a single injectable using Segment Anything Model (SAM)
-----------------------------------------------------------------------
Requirements:
    pip install torch torchvision segment-anything opencv-python matplotlib

Download the SAM checkpoint first:
    wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth

Usage:
    python segment_injectable.py --image your_image.jpg
"""

import argparse
import cv2
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from segment_anything import sam_model_registry, SamAutomaticMaskGenerator

# ── CONFIG ──────────────────────────────────────────────────────────────────
SAM_CHECKPOINT = "sam_vit_h_4b8939.pth"   # path to downloaded weights
MODEL_TYPE     = "vit_h"                   # matches the checkpoint above
DEVICE         = "cpu"                     # change to "cuda" if you have a GPU

# Tuning knobs — adjust these if SAM picks up noise or misses the injectable
MIN_MASK_AREA_FRACTION = 0.002   # ignore masks smaller than 0.2% of image area
MAX_MASK_AREA_FRACTION = 0.50    # ignore masks larger than 50% (e.g. whole background)
# ────────────────────────────────────────────────────────────────────────────


def load_image(path: str) -> np.ndarray:
    """Load image as RGB numpy array."""
    bgr = cv2.imread(path)
    if bgr is None:
        raise FileNotFoundError(f"Could not open image: {path}")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def run_sam(image: np.ndarray):
    """Run SAM in automatic mode and return all masks."""
    print("Loading SAM model…")
    sam = sam_model_registry[MODEL_TYPE](checkpoint=SAM_CHECKPOINT)
    sam.to(device=DEVICE)

    generator = SamAutomaticMaskGenerator(
        model=sam,
        points_per_side=32,           # denser grid = more thorough but slower
        pred_iou_thresh=0.88,         # keep only high-confidence masks
        stability_score_thresh=0.95,
        crop_n_layers=1,              # helps catch small objects
        crop_n_points_downscale_factor=2,
    )

    print("Generating masks…")
    masks = generator.generate(image)
    print(f"  SAM found {len(masks)} raw masks")
    return masks


BORDER_MARGIN = 10   # px — masks touching within this margin of the edge are background

def touches_border(mask_data, image_hw):
    """Return True if the mask's bounding box grazes the image edge."""
    h, w = image_hw
    x, y, bw, bh = mask_data["bbox"]
    return (x <= BORDER_MARGIN or y <= BORDER_MARGIN or
            x + bw >= w - BORDER_MARGIN or y + bh >= h - BORDER_MARGIN)


def filter_masks(masks, image_hw):
    """Keep only masks that look like an injectable."""
    h, w = image_hw
    total_pixels = h * w

    kept = []
    for m in masks:
        frac = m["area"] / total_pixels
        if frac < MIN_MASK_AREA_FRACTION or frac > MAX_MASK_AREA_FRACTION:
            continue
        if touches_border(m, (h, w)):
            continue                      # ← rejects background slabs
        kept.append(m)

    kept.sort(key=lambda m: m["predicted_iou"], reverse=True)
    print(f"  After size + border filter: {len(kept)} candidates")
    return kept


def score_mask(m, image_hw):
    """
    Score a mask on how injectable-like it is.
    Injectables are:
      - Long and thin  (high aspect ratio, target ~3–8×)
      - Not tiny       (minimum bbox long-side in pixels)
      - Reasonably confident
    """
    h, w = image_hw
    x, y, bw, bh = m["bbox"]
    long_side  = max(bw, bh)
    short_side = min(bw, bh) or 1
    ar = long_side / short_side

    # Penalise aspect ratios that are too extreme (>12) or too square (<2)
    ar_score = ar if 2 <= ar <= 12 else 0

    # Reward larger objects (real injectables span a good chunk of the frame)
    size_score = min(long_side / (max(h, w) * 0.5), 1.0)

    iou_score = m["predicted_iou"]

    return ar_score * 0.5 + size_score * 0.3 + iou_score * 0.2


def pick_best_mask(masks, image_hw=(None, None)):
    """Pick the most injectable-like mask using a composite score."""
    if not masks:
        return None

    scored = [(score_mask(m, image_hw), m) for m in masks]
    scored.sort(reverse=True)

    # Print top 3 for debugging
    for i, (sc, m) in enumerate(scored[:3]):
        x, y, bw, bh = m["bbox"]
        ar = max(bw, bh) / (min(bw, bh) or 1)
        print(f"  Candidate {i+1}: score={sc:.3f}  AR={ar:.1f}  bbox={m['bbox']}  IoU={m['predicted_iou']:.2f}")

    return scored[0][1]


def overlay_mask(image: np.ndarray, mask_data: dict) -> np.ndarray:
    """Blend the binary mask onto the image in semi-transparent green."""
    overlay = image.copy()
    binary  = mask_data["segmentation"]          # boolean H×W array
    overlay[binary] = (overlay[binary] * 0.4 + np.array([0, 220, 100]) * 0.6).astype(np.uint8)
    return overlay


def draw_bbox(ax, mask_data, color="lime"):
    x, y, w, h = mask_data["bbox"]
    rect = patches.Rectangle(
        (x, y), w, h,
        linewidth=2, edgecolor=color, facecolor="none"
    )
    ax.add_patch(rect)
    ax.text(x, y - 6, f"injectable  IoU={mask_data['predicted_iou']:.2f}",
            color=color, fontsize=9, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.2", fc="black", alpha=0.5))


def compute_grasp_center(mask_data):
    """
    Returns (cx, cy) — the centroid of the mask in image pixels.
    This is the 2-D point you'd aim the robot end-effector at (before depth).
    """
    binary = mask_data["segmentation"]
    ys, xs = np.where(binary)
    return int(xs.mean()), int(ys.mean())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True, help="Path to input image")
    parser.add_argument("--save",  default="result.png", help="Path to save result image")
    args = parser.parse_args()

    image = load_image(args.image)
    h, w  = image.shape[:2]
    print(f"Image size: {w}×{h} px")

    masks    = run_sam(image)
    filtered = filter_masks(masks, (h, w))
    best     = pick_best_mask(filtered, image_hw=(h, w))

    if best is None:
        print("No injectable detected. Try loosening MIN/MAX_MASK_AREA_FRACTION.")
        return

    cx, cy = compute_grasp_center(best)
    print(f"\n✓ Best mask — IoU: {best['predicted_iou']:.3f}")
    print(f"  Bounding box : {best['bbox']}")
    print(f"  Grasp center : ({cx}, {cy}) px  ← send this to your robot arm")

    # ── Visualise ────────────────────────────────────────────────────────────
    overlay = overlay_mask(image, best)

    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    axes[0].imshow(image);   axes[0].set_title("Original");  axes[0].axis("off")
    axes[1].imshow(overlay);                                  axes[1].axis("off")
    axes[1].set_title("Detected injectable")
    draw_bbox(axes[1], best)
    axes[1].plot(cx, cy, "r+", markersize=14, markeredgewidth=2)  # grasp point

    plt.tight_layout()
    plt.savefig(args.save, dpi=150)
    print(f"\nResult saved to: {args.save}")
    plt.show()


if __name__ == "__main__":
    main()