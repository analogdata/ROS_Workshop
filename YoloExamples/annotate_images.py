"""
Local Image Annotation Tool for YOLO Training
===============================================
A simple OpenCV-based tool to draw bounding boxes on images and save
labels in YOLO format — no internet, no Roboflow, no external tools.

What is Annotation?
  Annotation (or "labeling") means drawing bounding boxes around objects
  in images and assigning a class name to each box. This labeled data is
  what YOLO learns from during training.

YOLO Label Format:
  Each image gets a .txt file with the same name. Each line in the file
  represents one object:

    <class_id> <center_x> <center_y> <width> <height>

  All values are normalized (0.0 to 1.0) relative to image dimensions.
  Example: "0 0.5 0.5 0.3 0.4" means class 0, centered at (50%, 50%),
  width 30%, height 40% of the image.

How to Use:
  1. Put your images in a folder (e.g., YoloExamples/my_dataset/images/)
  2. Define your class names in the CLASSES list below
  3. Run: uv run python YoloExamples/annotate_images.py
  4. Draw boxes, assign classes, navigate between images
  5. Labels are saved automatically in YOLO format

Controls:
  Mouse:
    Left-click + drag  — Draw a bounding box
    Right-click        — Delete the nearest box

  Keyboard:
    1-9        — Select class (1 = first class, 2 = second, etc.)
    n / →      — Next image
    p / ←      — Previous image
    s          — Save labels for current image
    u          — Undo last box on current image
    c          — Clear all boxes on current image
    h          — Show/hide help overlay
    q / Esc    — Save and quit

Usage:
  # Annotate images in a folder:
  uv run python YoloExamples/annotate_images.py --images path/to/images/

  # Specify custom classes:
  uv run python YoloExamples/annotate_images.py --images path/to/images/ --classes cat dog bird

  # Resume annotation (loads existing labels):
  uv run python YoloExamples/annotate_images.py --images path/to/images/ --labels path/to/labels/
"""

# ─── Imports ──────────────────────────────────────────────────────────────
import cv2
import os
import glob
import argparse


# ─── Configuration ────────────────────────────────────────────────────────
# Default classes — change these to match YOUR objects.
# When you run the tool, you can also pass --classes on the command line.
DEFAULT_CLASSES = ["object"]

# Colors for each class (BGR format for OpenCV)
# We cycle through these if you have more classes than colors.
COLORS = [
    (0, 255, 0),      # Green
    (255, 0, 0),      # Blue
    (0, 0, 255),      # Red
    (255, 255, 0),    # Cyan
    (0, 255, 255),    # Yellow
    (255, 0, 255),    # Magenta
    (128, 255, 0),    # Lime
    (255, 128, 0),    # Light blue
    (0, 128, 255),    # Orange
    (255, 0, 128),    # Pink
]

# Supported image extensions
IMAGE_EXTENSIONS = [
    "*.jpg", "*.jpeg", "*.png", "*.bmp", "*.tiff", "*.webp"
]

# Window name
WINDOW_NAME = "YOLO Annotation Tool"


# ─── Annotation State ────────────────────────────────────────────────────
# This class holds all the state for the current annotation session.
# It tracks the current image, all bounding boxes, the selected class,
# and the mouse drawing state.
class AnnotationState:
    """Holds the state of the annotation session."""

    def __init__(self, image_paths, classes, labels_dir):
        # ── Image list ──
        self.image_paths = image_paths
        self.current_index = 0

        # ── Classes ──
        self.classes = classes
        self.current_class = 0  # Index into self.classes

        # ── Labels directory ──
        # Labels are saved as .txt files alongside images or in a
        # separate directory.
        self.labels_dir = labels_dir

        # ── Bounding boxes for ALL images ──
        # Key: image_path, Value: list of (class_id, x1, y1, x2, y2)
        # These are in PIXEL coordinates (not normalized).
        self.boxes = {}

        # ── Mouse drawing state ──
        self.drawing = False
        self.start_x = 0
        self.start_y = 0
        self.end_x = 0
        self.end_y = 0

        # ── UI state ──
        self.show_help = True
        self.unsaved_changes = False

        # ── Load existing labels ──
        for path in self.image_paths:
            self.boxes[path] = []
            self._load_labels(path)

    @property
    def current_path(self):
        """Get the current image path."""
        return self.image_paths[self.current_index]

    @property
    def current_boxes(self):
        """Get bounding boxes for the current image."""
        return self.boxes[self.current_path]

    def _label_path_for(self, image_path):
        """
        Get the label .txt file path for a given image.
        If a labels_dir is specified, put labels there.
        Otherwise, put them next to the image.
        """
        base = os.path.splitext(os.path.basename(image_path))[0]
        if self.labels_dir:
            return os.path.join(self.labels_dir, base + ".txt")
        else:
            img_dir = os.path.dirname(image_path)
            # Create a 'labels' folder next to 'images' if possible
            parent = os.path.dirname(img_dir)
            labels_dir = os.path.join(parent, "labels")
            if os.path.basename(img_dir).lower() == "images":
                os.makedirs(labels_dir, exist_ok=True)
                return os.path.join(labels_dir, base + ".txt")
            else:
                # Put labels next to images
                return os.path.join(img_dir, base + ".txt")

    def _load_labels(self, image_path):
        """
        Load existing YOLO-format labels for an image.
        YOLO format: <class_id> <cx> <cy> <w> <h> (normalized 0-1)
        We convert back to pixel coordinates for display.
        """
        label_path = self._label_path_for(image_path)
        if not os.path.exists(label_path):
            return

        # We need image dimensions to convert normalized → pixel
        img = cv2.imread(image_path)
        if img is None:
            return
        h, w = img.shape[:2]

        with open(label_path, "r") as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) != 5:
                    continue
                class_id = int(parts[0])
                cx = float(parts[1]) * w
                cy = float(parts[2]) * h
                bw = float(parts[3]) * w
                bh = float(parts[4]) * h

                x1 = int(cx - bw / 2)
                y1 = int(cy - bh / 2)
                x2 = int(cx + bw / 2)
                y2 = int(cy + bh / 2)

                self.boxes[image_path].append(
                    (class_id, x1, y1, x2, y2)
                )

    def save_labels(self, image_path=None):
        """
        Save bounding boxes for an image in YOLO format.
        Converts pixel coordinates → normalized (0-1) coordinates.
        """
        if image_path is None:
            image_path = self.current_path

        img = cv2.imread(image_path)
        if img is None:
            return
        h, w = img.shape[:2]

        label_path = self._label_path_for(image_path)
        os.makedirs(os.path.dirname(label_path), exist_ok=True)

        with open(label_path, "w") as f:
            for (class_id, x1, y1, x2, y2) in self.boxes[image_path]:
                # Convert pixel → normalized YOLO format
                cx = ((x1 + x2) / 2.0) / w
                cy = ((y1 + y2) / 2.0) / h
                bw = abs(x2 - x1) / w
                bh = abs(y2 - y1) / h

                # Clamp to [0, 1]
                cx = max(0.0, min(1.0, cx))
                cy = max(0.0, min(1.0, cy))
                bw = max(0.0, min(1.0, bw))
                bh = max(0.0, min(1.0, bh))

                f.write(
                    f"{class_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n"
                )

        self.unsaved_changes = False
        print(
            f"  Saved {len(self.boxes[image_path])} labels → "
            f"{label_path}"
        )

    def save_all(self):
        """Save labels for all images."""
        for path in self.image_paths:
            if self.boxes[path]:  # Only save if there are boxes
                self.save_labels(path)

    def add_box(self, class_id, x1, y1, x2, y2):
        """Add a bounding box to the current image."""
        # Ensure x1 < x2 and y1 < y2
        x1, x2 = min(x1, x2), max(x1, x2)
        y1, y2 = min(y1, y2), max(y1, y2)

        # Ignore tiny boxes (accidental clicks)
        if abs(x2 - x1) < 5 or abs(y2 - y1) < 5:
            return

        self.current_boxes.append((class_id, x1, y1, x2, y2))
        self.unsaved_changes = True

    def undo_last_box(self):
        """Remove the last bounding box from the current image."""
        if self.current_boxes:
            self.current_boxes.pop()
            self.unsaved_changes = True

    def clear_boxes(self):
        """Remove all bounding boxes from the current image."""
        self.boxes[self.current_path] = []
        self.unsaved_changes = True

    def delete_nearest_box(self, x, y):
        """Delete the box whose center is nearest to (x, y)."""
        if not self.current_boxes:
            return

        min_dist = float("inf")
        min_idx = -1

        for i, (_, bx1, by1, bx2, by2) in enumerate(self.current_boxes):
            cx = (bx1 + bx2) / 2
            cy = (by1 + by2) / 2
            dist = ((cx - x) ** 2 + (cy - y) ** 2) ** 0.5
            if dist < min_dist:
                min_dist = dist
                min_idx = i

        if min_idx >= 0:
            self.current_boxes.pop(min_idx)
            self.unsaved_changes = True

    def next_image(self):
        """Go to the next image (auto-saves current)."""
        if self.unsaved_changes:
            self.save_labels()
        self.current_index = (
            (self.current_index + 1) % len(self.image_paths)
        )
        self.drawing = False

    def prev_image(self):
        """Go to the previous image (auto-saves current)."""
        if self.unsaved_changes:
            self.save_labels()
        self.current_index = (
            (self.current_index - 1) % len(self.image_paths)
        )
        self.drawing = False


# ─── Mouse Callback ──────────────────────────────────────────────────────
# OpenCV calls this function whenever the mouse does something in the
# window. We use it to track click-and-drag for drawing bounding boxes.
def mouse_callback(event, x, y, flags, param):
    """Handle mouse events for drawing bounding boxes."""
    state = param

    if event == cv2.EVENT_LBUTTONDOWN:
        # Start drawing a new box
        state.drawing = True
        state.start_x = x
        state.start_y = y
        state.end_x = x
        state.end_y = y

    elif event == cv2.EVENT_MOUSEMOVE:
        # Update the box preview while dragging
        if state.drawing:
            state.end_x = x
            state.end_y = y

    elif event == cv2.EVENT_LBUTTONUP:
        # Finish drawing — add the box
        state.drawing = False
        state.end_x = x
        state.end_y = y
        state.add_box(
            state.current_class,
            state.start_x, state.start_y,
            state.end_x, state.end_y
        )

    elif event == cv2.EVENT_RBUTTONDOWN:
        # Right-click deletes the nearest box
        state.delete_nearest_box(x, y)


# ─── Drawing Functions ────────────────────────────────────────────────────
def draw_boxes(img, state):
    """Draw all bounding boxes and labels on the image."""
    for (class_id, x1, y1, x2, y2) in state.current_boxes:
        color = COLORS[class_id % len(COLORS)]
        class_name = (
            state.classes[class_id]
            if class_id < len(state.classes)
            else f"class_{class_id}"
        )

        # Draw the bounding box
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

        # Draw the label background
        label = f"{class_name} ({class_id})"
        (tw, th), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
        )
        cv2.rectangle(
            img, (x1, y1 - th - 8), (x1 + tw + 4, y1), color, -1
        )
        cv2.putText(
            img, label, (x1 + 2, y1 - 4),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
        )

    # Draw the box being drawn (preview)
    if state.drawing:
        color = COLORS[state.current_class % len(COLORS)]
        cv2.rectangle(
            img,
            (state.start_x, state.start_y),
            (state.end_x, state.end_y),
            color, 1
        )


def draw_info_bar(img, state):
    """Draw the info bar at the top of the image."""
    h, w = img.shape[:2]

    # Semi-transparent black bar at the top
    overlay = img.copy()
    cv2.rectangle(overlay, (0, 0), (w, 40), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.7, img, 0.3, 0, img)

    # Image counter
    counter = (
        f"Image {state.current_index + 1}/{len(state.image_paths)}"
    )
    cv2.putText(
        img, counter, (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1
    )

    # Current class
    class_name = state.classes[state.current_class]
    class_color = COLORS[state.current_class % len(COLORS)]
    class_text = f"Class: {class_name} [{state.current_class}]"
    cv2.putText(
        img, class_text, (250, 28),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, class_color, 2
    )

    # Box count
    box_count = f"Boxes: {len(state.current_boxes)}"
    cv2.putText(
        img, box_count, (550, 28),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1
    )

    # Unsaved indicator
    if state.unsaved_changes:
        cv2.putText(
            img, "[UNSAVED]", (700, 28),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2
        )

    # Filename
    filename = os.path.basename(state.current_path)
    cv2.putText(
        img, filename, (10, h - 10),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1
    )


def draw_help_overlay(img, state):
    """Draw a help overlay showing all controls."""
    h, w = img.shape[:2]

    # Semi-transparent background
    overlay = img.copy()
    cv2.rectangle(overlay, (w // 4, 50), (3 * w // 4, h - 50),
                  (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.85, img, 0.15, 0, img)

    x_start = w // 4 + 20
    y_start = 90
    line_h = 28

    lines = [
        ("=== YOLO Annotation Tool ===", (0, 255, 255)),
        ("", (255, 255, 255)),
        ("MOUSE:", (0, 255, 0)),
        ("  Left-click + drag  = Draw box", (255, 255, 255)),
        ("  Right-click        = Delete nearest box", (255, 255, 255)),
        ("", (255, 255, 255)),
        ("KEYBOARD:", (0, 255, 0)),
        ("  1-9    = Select class", (255, 255, 255)),
        ("  n / -> = Next image", (255, 255, 255)),
        ("  p / <- = Previous image", (255, 255, 255)),
        ("  s      = Save labels", (255, 255, 255)),
        ("  u      = Undo last box", (255, 255, 255)),
        ("  c      = Clear all boxes", (255, 255, 255)),
        ("  h      = Toggle this help", (255, 255, 255)),
        ("  q/Esc  = Save and quit", (255, 255, 255)),
        ("", (255, 255, 255)),
        ("CLASSES:", (0, 255, 0)),
    ]

    # Add class list
    for i, cls in enumerate(state.classes):
        color = COLORS[i % len(COLORS)]
        marker = " <--" if i == state.current_class else ""
        lines.append((f"  [{i}] {cls}{marker}", color))

    lines.append(("", (255, 255, 255)))
    lines.append(("Press 'h' to hide this help", (128, 128, 128)))

    for i, (text, color) in enumerate(lines):
        if text:
            cv2.putText(
                img, text, (x_start, y_start + i * line_h),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1
            )


# ─── Main Function ────────────────────────────────────────────────────────
def main():
    """Main annotation loop."""
    # ── Parse command-line arguments ──
    parser = argparse.ArgumentParser(
        description="Local YOLO image annotation tool"
    )
    parser.add_argument(
        "--images", type=str, default=None,
        help="Path to folder containing images to annotate"
    )
    parser.add_argument(
        "--labels", type=str, default=None,
        help="Path to folder for saving/loading label .txt files "
             "(default: auto-detect)"
    )
    parser.add_argument(
        "--classes", nargs="+", default=None,
        help="List of class names (e.g., --classes cat dog bird)"
    )
    args = parser.parse_args()

    # ── Get image directory ──
    if args.images:
        image_dir = args.images
    else:
        # Default: look for a my_dataset/images folder
        default_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "my_dataset", "images"
        )
        if os.path.exists(default_dir):
            image_dir = default_dir
        else:
            print("=" * 60)
            print("  YOLO Annotation Tool")
            print("=" * 60)
            print()
            print("No image directory specified!")
            print()
            print("Usage:")
            print("  uv run python YoloExamples/annotate_images.py "
                  "--images path/to/images/")
            print()
            print("Or create the default directory:")
            print(f"  mkdir -p {default_dir}")
            print("  # Put your images there, then run again")
            print()
            return

    # ── Collect all images ──
    image_paths = []
    for ext in IMAGE_EXTENSIONS:
        image_paths.extend(
            glob.glob(os.path.join(image_dir, ext))
        )
        # Also check uppercase extensions
        image_paths.extend(
            glob.glob(os.path.join(image_dir, ext.upper()))
        )

    image_paths = sorted(set(image_paths))

    if not image_paths:
        print(f"No images found in: {image_dir}")
        print(f"Supported formats: {IMAGE_EXTENSIONS}")
        return

    # ── Set up classes ──
    classes = args.classes if args.classes else DEFAULT_CLASSES

    # ── Create annotation state ──
    state = AnnotationState(image_paths, classes, args.labels)

    print("=" * 60)
    print("  YOLO Annotation Tool")
    print("=" * 60)
    print(f"  Images:  {len(image_paths)} found in {image_dir}")
    print(f"  Classes: {classes}")
    print(f"  Labels:  {state._label_path_for(image_paths[0])}")
    print()
    print("  Press 'h' to toggle help overlay")
    print("  Press 'q' or Esc to save and quit")
    print("=" * 60)

    # ── Create window and set mouse callback ──
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WINDOW_NAME, mouse_callback, state)

    # ── Main loop ──
    while True:
        # Load the current image
        img = cv2.imread(state.current_path)
        if img is None:
            print(f"Could not read: {state.current_path}")
            state.next_image()
            continue

        # Draw everything on a copy (don't modify the original)
        display = img.copy()
        draw_boxes(display, state)
        draw_info_bar(display, state)

        if state.show_help:
            draw_help_overlay(display, state)

        # Show the image
        cv2.imshow(WINDOW_NAME, display)

        # ── Handle keyboard input ──
        # waitKey(30) waits 30ms for a key press. This gives ~33 FPS
        # for smooth box-drawing preview.
        key = cv2.waitKey(30) & 0xFF

        if key == 255:
            # No key pressed
            continue

        # Quit
        if key == ord("q") or key == 27:  # 'q' or Esc
            if state.unsaved_changes:
                state.save_labels()
            print("\nSaving all labels and exiting...")
            state.save_all()
            break

        # Next image
        elif key == ord("n") or key == 83:  # 'n' or right arrow
            state.next_image()

        # Previous image
        elif key == ord("p") or key == 81:  # 'p' or left arrow
            state.prev_image()

        # Save
        elif key == ord("s"):
            state.save_labels()
            print("  Labels saved!")

        # Undo
        elif key == ord("u"):
            state.undo_last_box()

        # Clear
        elif key == ord("c"):
            state.clear_boxes()

        # Toggle help
        elif key == ord("h"):
            state.show_help = not state.show_help

        # Number keys 1-9 to select class
        elif ord("1") <= key <= ord("9"):
            class_idx = key - ord("1")  # '1' → 0, '2' → 1, etc.
            if class_idx < len(classes):
                state.current_class = class_idx
                print(
                    f"  Selected class: "
                    f"{classes[class_idx]} [{class_idx}]"
                )

    # ── Cleanup ──
    cv2.destroyAllWindows()
    print("Annotation complete!")


# ─── Entry Point ──────────────────────────────────────────────────────────
# Run this file directly:
#   uv run python YoloExamples/annotate_images.py --images path/to/images/
if __name__ == "__main__":
    main()
