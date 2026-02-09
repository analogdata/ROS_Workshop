# Annotating & Training Custom YOLO Models — Complete Guide

## Table of Contents

1. [Overview — The Training Pipeline](#1-overview--the-training-pipeline)
2. [Public Datasets You Can Download Right Now](#2-public-datasets-you-can-download-right-now)
3. [Understanding the YOLO Label Format](#3-understanding-the-yolo-label-format)
4. [Setting Up Your Dataset Structure](#4-setting-up-your-dataset-structure)
5. [Annotating Images — How It Works](#5-annotating-images--how-it-works)
6. [Using Our Local Annotation Tool (annotate_images.py)](#6-using-our-local-annotation-tool-annotate_imagespy)
7. [Using labelImg (Classic Desktop Tool)](#7-using-labelimg-classic-desktop-tool)
8. [Using the Jupyter Notebook](#8-using-the-jupyter-notebook)
9. [Training — How It Works Under the Hood](#9-training--how-it-works-under-the-hood)
10. [Training Commands — Step by Step](#10-training-commands--step-by-step)
11. [Evaluating Your Model](#11-evaluating-your-model)
12. [Predicting with Your Trained Model](#12-predicting-with-your-trained-model)
13. [Exporting for Deployment](#13-exporting-for-deployment)
14. [Complete Walkthrough: Helmet Detection](#14-complete-walkthrough-helmet-detection)
15. [Complete Walkthrough: Face Mask Detection](#15-complete-walkthrough-face-mask-detection)
16. [Tips for Better Models](#16-tips-for-better-models)
17. [Troubleshooting](#17-troubleshooting)
18. [All Commands Quick Reference](#18-all-commands-quick-reference)

---

## 1. Overview — The Training Pipeline

Training a custom YOLO model follows this pipeline:

```
┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
│ Collect  │    │ Annotate │    │ Organize │    │  Train   │    │  Deploy  │
│ Images   │───>│ (Label)  │───>│ Dataset  │───>│  Model   │───>│ & Test   │
│          │    │          │    │          │    │          │    │          │
│ Camera,  │    │ Draw     │    │ train/   │    │ 50-100   │    │ Webcam,  │
│ download │    │ bounding │    │ val/     │    │ epochs   │    │ images,  │
│ dataset  │    │ boxes    │    │ data.yaml│    │          │    │ ESP8266  │
└──────────┘    └──────────┘    └──────────┘    └──────────┘    └──────────┘
```

**What you need:**
- Images of the objects you want to detect (50-500+)
- Labels (bounding boxes drawn around each object)
- A `data.yaml` file telling YOLO where to find everything
- A base model to start from (`yolov8n.pt`)

**What you get:**
- A custom `.pt` model file that detects YOUR specific objects
- Works with webcam, images, video, or connected to ESP8266

---

## 2. Public Datasets You Can Download Right Now

Instead of collecting and annotating images yourself, you can use
these **free, publicly available datasets** that are already labeled
in YOLO format (or easily convertible).

### Dataset 1: Hard Hat / Safety Helmet Detection

Detect whether construction workers are wearing helmets.

| Detail | Info |
|--------|------|
| **Classes** | `helmet`, `head` (no helmet) |
| **Images** | ~5,000 |
| **Source** | Roboflow Universe |
| **Format** | YOLO format (ready to use) |
| **License** | CC BY 4.0 (free for any use) |
| **Download** | [Hard Hat Workers Dataset](https://universe.roboflow.com/joseph-nelson/hard-hat-workers/dataset/5) |

**How to download:**

```bash
# Method 1: Using Roboflow (recommended — gives YOLO format directly)
# 1. Go to the link above
# 2. Click "Download Dataset"
# 3. Select "YOLOv8" format
# 4. Download the ZIP file
# 5. Extract into YoloExamples/my_dataset/

# Method 2: Using the Roboflow Python API (free account required)
pip install roboflow
python -c "
from roboflow import Roboflow
rf = Roboflow(api_key='YOUR_API_KEY')  # Get free key at roboflow.com
project = rf.workspace('joseph-nelson').project('hard-hat-workers')
version = project.version(5)
dataset = version.download('yolov8')
"
```

After downloading, your folder should look like:
```
Hard-Hat-Workers-5/
├── data.yaml
├── train/
│   ├── images/
│   └── labels/
├── valid/          ← Rename to 'val' or update data.yaml
│   ├── images/
│   └── labels/
└── test/
    ├── images/
    └── labels/
```

---

### Dataset 2: Face Mask Detection

Detect whether people are wearing face masks.

| Detail | Info |
|--------|------|
| **Classes** | `with_mask`, `without_mask`, `mask_worn_incorrectly` |
| **Images** | ~853 |
| **Source** | Kaggle |
| **Format** | YOLO format |
| **License** | Open |
| **Download** | [Face Mask Detection Dataset](https://www.kaggle.com/datasets/andrewmvd/face-mask-detection) |

**How to download:**

```bash
# Method 1: Direct from Kaggle (requires free Kaggle account)
# 1. Go to the link above
# 2. Click "Download" button
# 3. Extract the ZIP

# Method 2: Using Kaggle CLI
pip install kaggle
# Put your kaggle.json API key in ~/.kaggle/
kaggle datasets download -d andrewmvd/face-mask-detection
unzip face-mask-detection.zip -d YoloExamples/face_mask_dataset/
```

> **Note:** This dataset uses Pascal VOC XML format. You'll need to
> convert it to YOLO format. See the conversion script below.

---

### Dataset 3: Traffic Signs Detection

Detect various traffic signs (stop, yield, speed limit, etc.).

| Detail | Info |
|--------|------|
| **Classes** | `prohibitory`, `danger`, `mandatory`, `other` |
| **Images** | ~900 |
| **Source** | Roboflow Universe |
| **Format** | YOLO format |
| **License** | CC BY 4.0 |
| **Download** | [Road Sign Detection](https://universe.roboflow.com/roboflow-100/road-sign-detection-myu7o/dataset/2) |

---

### Dataset 4: Fire and Smoke Detection

Detect fire and smoke for safety monitoring.

| Detail | Info |
|--------|------|
| **Classes** | `fire`, `smoke` |
| **Images** | ~3,000 |
| **Source** | Roboflow Universe |
| **Format** | YOLO format |
| **License** | CC BY 4.0 |
| **Download** | [Fire and Smoke Dataset](https://universe.roboflow.com/fire-lfnag/fire-and-smoke-dntmr/dataset/1) |

---

### Dataset 5: Vehicle Detection (Cars, Trucks, Buses)

Detect vehicles on roads for traffic monitoring.

| Detail | Info |
|--------|------|
| **Classes** | `car`, `truck`, `bus`, `motorcycle`, `bicycle` |
| **Images** | ~2,000 |
| **Source** | Roboflow Universe |
| **Format** | YOLO format |
| **License** | CC BY 4.0 |
| **Download** | [Vehicle Detection Dataset](https://universe.roboflow.com/roboflow-100/vehicles-q0x2v/dataset/2) |

---

### Dataset 6: PCB Defect Detection (Electronics / Embedded)

Detect defects on printed circuit boards — great for embedded/IoT projects.

| Detail | Info |
|--------|------|
| **Classes** | `missing_hole`, `mouse_bite`, `open_circuit`, `short`, `spur`, `spurious_copper` |
| **Images** | ~693 |
| **Source** | Roboflow Universe |
| **Format** | YOLO format |
| **License** | CC BY 4.0 |
| **Download** | [PCB Defect Detection](https://universe.roboflow.com/dipesh-poudel-4bfkn/pcb-defect-detection-k0vqe/dataset/1) |

---

### Dataset 7: COCO128 (Quick Testing / Demo)

A tiny subset of the COCO dataset — perfect for testing your pipeline.

| Detail | Info |
|--------|------|
| **Classes** | 80 COCO classes (person, car, dog, etc.) |
| **Images** | 128 |
| **Source** | Ultralytics |
| **Format** | YOLO format |
| **License** | CC BY 4.0 |

**How to download:**

```bash
# Ultralytics downloads it automatically when you train with coco128.yaml
uv run yolo detect train model=yolov8n.pt data=coco128.yaml epochs=5

# Or download manually:
# https://github.com/ultralytics/assets/releases/download/v0.0.0/coco128.zip
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/coco128.zip
unzip coco128.zip -d YoloExamples/
```

---

### Where to Find More Datasets

| Platform | URL | Notes |
|----------|-----|-------|
| **Roboflow Universe** | [universe.roboflow.com](https://universe.roboflow.com) | 250,000+ datasets, most in YOLO format, free download |
| **Kaggle Datasets** | [kaggle.com/datasets](https://www.kaggle.com/datasets) | Huge collection, may need format conversion |
| **Google Open Images** | [storage.googleapis.com/openimages](https://storage.googleapis.com/openimages/web/index.html) | 9M images, 600 classes |
| **Papers With Code** | [paperswithcode.com/datasets](https://paperswithcode.com/datasets) | Academic datasets with benchmarks |
| **Hugging Face** | [huggingface.co/datasets](https://huggingface.co/datasets) | Growing collection, easy API |

> **Tip:** On Roboflow Universe, always select **"YOLOv8"** as the export
> format. This gives you the exact folder structure YOLO expects.

---

## 3. Understanding the YOLO Label Format

Every image needs a matching `.txt` label file. Understanding this
format is crucial for annotation and debugging.

### The Format

Each line in a `.txt` file represents **one object**:

```
<class_id> <center_x> <center_y> <width> <height>
```

All coordinates are **normalized** (0.0 to 1.0) relative to the image:

```
Example: "0 0.45 0.60 0.30 0.40"

Means:
  class_id = 0          (first class, e.g., "helmet")
  center_x = 0.45       (45% from the left edge)
  center_y = 0.60       (60% from the top edge)
  width    = 0.30       (box is 30% of image width)
  height   = 0.40       (box is 40% of image height)
```

### Visual Explanation

```
(0,0) ──────────────── 1.0 ──> x
  │
  │         center_x = 0.45
  │              │
  │    ┌─────────┼─────────┐
  │    │         │         │
  │    │    ┌────●────┐    │  center_y = 0.60
  │    │    │  helmet  │    │
  │    │    │  (0.30)  │    │  ← width = 0.30
  │    │    └─────────┘    │
  │    │     (0.40)        │  ← height = 0.40
  │    └───────────────────┘
  │
 1.0
  │
  ▼ y
```

### Why Normalized?

Normalized coordinates (0-1) work regardless of image resolution.
The same label works whether the image is 640×480 or 1920×1080.

### Converting Pixel Coordinates to YOLO Format

If you have pixel coordinates (x1, y1, x2, y2):

```python
# Image dimensions
img_width = 640
img_height = 480

# Pixel coordinates (top-left and bottom-right corners)
x1, y1, x2, y2 = 100, 150, 300, 350

# Convert to YOLO format
center_x = ((x1 + x2) / 2) / img_width    # 0.3125
center_y = ((y1 + y2) / 2) / img_height    # 0.5208
width    = (x2 - x1) / img_width           # 0.3125
height   = (y2 - y1) / img_height          # 0.4167

# YOLO label line:
# 0 0.312500 0.520833 0.312500 0.416667
```

### Multiple Objects in One Image

If an image has 3 objects, the label file has 3 lines:

```
# img001.txt
0 0.45 0.60 0.30 0.40
1 0.75 0.30 0.15 0.20
0 0.20 0.80 0.25 0.35
```

This means:
- Object 1: class 0 (helmet) at center (0.45, 0.60)
- Object 2: class 1 (no_helmet) at center (0.75, 0.30)
- Object 3: class 0 (helmet) at center (0.20, 0.80)

### No Objects in an Image?

If an image has no objects to detect, create an **empty** `.txt` file.
This tells YOLO "there's nothing here" — which is also useful training
data (teaches the model to NOT detect false positives).

---

## 4. Setting Up Your Dataset Structure

### The Required Structure

```
my_dataset/
├── data.yaml              ← Configuration file
├── train/
│   ├── images/            ← Training images (80%)
│   │   ├── img001.jpg
│   │   ├── img002.jpg
│   │   └── ...
│   └── labels/            ← Training labels (matching .txt files)
│       ├── img001.txt
│       ├── img002.txt
│       └── ...
└── val/
    ├── images/            ← Validation images (20%)
    │   ├── img050.jpg
    │   └── ...
    └── labels/            ← Validation labels
        ├── img050.txt
        └── ...
```

**Critical rules:**
- Each image MUST have a matching `.txt` file with the **same name**
- `img001.jpg` → `img001.txt`
- Images go in `images/`, labels go in `labels/`
- Train set = 80% of data, Val set = 20%

### The data.yaml File

```yaml
# data.yaml — Dataset configuration
# Paths are relative to this file's location

train: train/images
val: val/images

# Number of classes
nc: 2

# Class names (order = class_id)
# Index 0 = "helmet", Index 1 = "no_helmet"
names: ['helmet', 'no_helmet']
```

### Automated Setup Command

```bash
# Our script creates everything for you:
uv run python YoloExamples/train_custom_model.py \
    --setup --classes helmet no_helmet

# This creates:
#   YoloExamples/my_dataset/
#   ├── data.yaml          (auto-generated)
#   ├── train/images/      (empty, put images here)
#   ├── train/labels/      (empty, annotation tool fills this)
#   ├── val/images/        (empty, put images here)
#   └── val/labels/        (empty, annotation tool fills this)
```

### Using a Downloaded Dataset

If you downloaded a dataset from Roboflow:

```bash
# 1. Download and extract
unzip Hard-Hat-Workers-5.zip -d YoloExamples/helmet_dataset/

# 2. Check the structure
ls YoloExamples/helmet_dataset/
# Should see: data.yaml  train/  valid/  test/

# 3. Some datasets use 'valid' instead of 'val'
#    Check data.yaml and update if needed:
cat YoloExamples/helmet_dataset/data.yaml

# 4. Train directly using the dataset's data.yaml:
uv run python YoloExamples/train_custom_model.py --train
# (or modify the DATASET_DIR in the script)
```

---

## 5. Annotating Images — How It Works

### What is Annotation?

Annotation (also called "labeling") is the process of:
1. Looking at an image
2. Drawing a bounding box around each object
3. Assigning a class name to each box
4. Saving the box coordinates in YOLO format

```
Original Image:              Annotated Image:
┌──────────────────┐         ┌──────────────────┐
│                  │         │  ┌────────┐      │
│    Person with   │   ───>  │  │ helmet │      │
│    a helmet      │         │  │  0     │      │
│                  │         │  └────────┘      │
│                  │         │                  │
└──────────────────┘         └──────────────────┘

Saved as img001.txt:
0 0.35 0.25 0.20 0.15
```

### Annotation Best Practices

1. **Draw tight boxes** — The box should fit snugly around the object,
   not include lots of background.

```
Good:                    Bad:
┌──────┐                 ┌──────────────┐
│helmet│                 │              │
│      │                 │   helmet     │
└──────┘                 │              │
                         │              │
                         └──────────────┘
  Tight fit               Too much padding
```

2. **Label ALL objects** — If there are 5 helmets in an image, draw
   5 boxes. Missing labels confuse the model.

3. **Be consistent** — If you label a partially visible helmet in one
   image, do the same in all images.

4. **Include negative examples** — Some images with NO objects help
   the model learn what is NOT a detection.

5. **Label edge cases** — Include occluded (partially hidden), small,
   and distant objects. These are the hardest to detect.

### Our Annotation Tools

| Tool | Best for | Command |
|------|----------|---------|
| **annotate/app.py** (Flask) | Browser-based UI, like Roboflow — folder browser, upload, in-app classes | `uv run python YoloExamples/annotate/app.py` |
| **annotate_images.py** (OpenCV) | Lightweight CLI, no browser needed | `uv run python YoloExamples/annotate_images.py` |
| **Jupyter Notebook** | Interactive workflow with visualization | `uv run jupyter lab YoloExamples/yolo_training_workflow.ipynb` |
| **labelImg** | Feature-rich desktop tool | `pip install labelImg && labelImg` |

---

## 6. Using the Web Annotation Tool (Recommended)

Our primary annotation tool runs in your browser — just like Roboflow,
but 100% local. No account, no upload, no internet needed.

### Starting the Tool

```bash
uv run python YoloExamples/annotate/app.py
# Opens at http://localhost:5000
```

No flags needed — everything is configured in the web UI.

### Home Page

The home page gives you two options:

- **Select Folder** — Browse your filesystem and pick a folder of images
- **Upload Images** — Drag & drop or select images to create a new project

![Annotator Home Page](Annotator%20Tool/1.png)

Both options let you define classes before you start annotating.

Clicking **Select Folder** opens the folder browser where you can navigate
to your images directory, see how many images were found, define your
classes, and click **Start Annotating**:

![Annotator Folder Browser](Annotator%20Tool/2.png)

### Annotation Page

The annotation page has a **sidebar** (left) and a **canvas** (center):

- **Sidebar:** Class selector (add new classes on-the-fly), progress bar,
  save/export buttons
- **Canvas:** Click & drag to draw bounding boxes, right-click to delete
- **Box list:** Shows all annotations with YOLO coordinates and delete buttons

![Annotator Annotation Page](Annotator%20Tool/3.png)

### How to Use

1. **Load images:** Choose "Select Folder" or "Upload Images" on the home page
2. **Add classes:** Type class names in the modal or in the annotation sidebar
3. **Select active class:** Click the class button or press 1-9
4. **Draw boxes:** Click and drag on the image to draw a bounding box
5. **Navigate:** Use ← Prev / Next → buttons or A/D keys (auto-saves!)
6. **Delete a box:** Right-click on canvas or click ✕ in the box list
7. **Save:** Click "💾 Save Labels" or press S. Download all as ZIP.

### Multi-Class Labeling (Multiple Objects per Image)

YOLO supports **multiple objects with different classes in the same image**.
Each bounding box becomes one line in the label `.txt` file.

**Example:** An image with 2 people — one wearing a helmet, one without:

```
0 0.4807 0.2089 0.3461 0.3750    ← class 0 (helmet)
1 0.7200 0.5100 0.2000 0.3000    ← class 1 (no_helmet)
```

Each line follows the format: `<class_id> <center_x> <center_y> <width> <height>`
(all coordinates normalized 0.0–1.0 relative to image dimensions).

**How to annotate multi-class images:**

1. Select class `helmet` in the sidebar (or press `1`)
2. Draw a box around the person wearing a helmet
3. Switch to class `no_helmet` (click it or press `2`)
4. Draw a box around the person without a helmet
5. Press `D` or `→` to move to the next image (auto-saves)

You can draw **as many boxes as needed** per image, mixing any classes.
The tool saves all of them into a single `.txt` file for that image.

**Key points:**
- Class order matters: class 0 = first line in `classes.txt`, class 1 = second, etc.
- An image can have 0 boxes (negative example), 1 box, or many boxes
- Different images can have different combinations of classes
- The `classes.txt` and `annotation_meta.json` files track your class definitions

### Features

- **Browser-based** — Works on any OS, no OpenCV window issues
- **Folder browser** — Navigate your filesystem in the UI
- **Image upload** — Drag & drop images to create a new project
- **In-app class management** — Add classes anytime, saved as metadata
- **Visual class colors** — Each class gets a distinct color
- **Progress tracker** — See how many images you've labeled
- **Auto-save** — Labels save automatically when you navigate
- **Metadata per folder** — `annotation_meta.json` + `classes.txt` in labels dir
- **ZIP export** — Download all labels as a ZIP file
- **Resume support** — Existing labels and classes load automatically
- **Keyboard shortcuts** — 1-9 for class, A/D for nav, Z for undo, S for save
- **Usage / Help page** — Built-in documentation at `/help`

### Keyboard Shortcuts (Web Annotator)

| Key | Action |
|-----|--------|
| `1` – `9` | Select class by number |
| `D` / `→` | Next image (auto-saves) |
| `A` / `←` | Previous image (auto-saves) |
| `S` | Save labels |
| `Z` | Undo last box |
| `C` | Clear all boxes |
| Right-click | Delete nearest box |

---

## 6b. Using the CLI Annotation Tool (annotate_images.py)

A lightweight alternative that uses OpenCV — no browser needed.

### Starting the Tool

```bash
# Basic usage — annotate images in a folder
uv run python YoloExamples/annotate_images.py \
    --images YoloExamples/my_dataset/train/images/ \
    --classes helmet no_helmet

# With a custom labels directory
uv run python YoloExamples/annotate_images.py \
    --images YoloExamples/my_dataset/train/images/ \
    --labels YoloExamples/my_dataset/train/labels/ \
    --classes helmet no_helmet

# With more classes
uv run python YoloExamples/annotate_images.py \
    --images path/to/images/ \
    --classes cat dog bird fish
```

### Controls

```
┌─────────────────────────────────────────────────────────┐
│                  ANNOTATION CONTROLS                    │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  MOUSE:                                                 │
│    Left-click + drag  → Draw a bounding box             │
│    Right-click        → Delete the nearest box          │
│                                                         │
│  KEYBOARD:                                              │
│    1-9                → Select class (1=first, 2=second)│
│    n  or  →           → Next image (auto-saves)         │
│    p  or  ←           → Previous image (auto-saves)     │
│    s                  → Save labels for current image   │
│    u                  → Undo last box                   │
│    c                  → Clear all boxes on this image   │
│    h                  → Show/hide help overlay          │
│    q  or  Esc         → Save everything and quit        │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### What the Screen Looks Like

```
┌─────────────────────────────────────────────────────────┐
│ Image 3/50 │ Class: helmet [0] │ Boxes: 2 │ [UNSAVED]  │  ← Info bar
├─────────────────────────────────────────────────────────┤
│                                                         │
│         ┌──────────┐                                    │
│         │ helmet(0)│                                    │
│         │          │      ┌─────────────┐               │
│         │          │      │ no_helmet(1)│               │
│         └──────────┘      │             │               │
│                           └─────────────┘               │
│                                                         │
│                                                         │
├─────────────────────────────────────────────────────────┤
│ img003.jpg                                              │  ← Filename
└─────────────────────────────────────────────────────────┘
```

### Workflow

```
1. Start the tool with your images and classes
2. For each image:
   a. Press 1-9 to select the class you want to label
   b. Left-click and drag to draw a box around each object
   c. Repeat for all objects in the image
   d. Press 'n' to go to the next image (auto-saves)
3. When done, press 'q' to save and quit
4. Labels are saved as .txt files in YOLO format
```

### Resuming Annotation

If you quit and come back later, the tool **automatically loads
existing labels**. Just run the same command again — your previous
boxes will appear on each image.

---

## 7. Using labelImg (Classic Desktop Tool)

[labelImg](https://github.com/HumanSignal/labelImg) is a popular
open-source annotation tool with more features than our simple tool.

### Install

```bash
pip install labelImg
```

### Run

```bash
# Open with a specific image directory and save format
labelImg YoloExamples/my_dataset/train/images/ \
         YoloExamples/my_dataset/classes.txt \
         YoloExamples/my_dataset/train/labels/
```

Create a `classes.txt` file first:
```bash
# Create classes.txt (one class per line)
echo -e "helmet\nno_helmet" > YoloExamples/my_dataset/classes.txt
```

### labelImg Controls

| Key | Action |
|-----|--------|
| `W` | Create a new bounding box |
| `D` | Next image |
| `A` | Previous image |
| `Ctrl+S` | Save |
| `Del` | Delete selected box |

> **Important:** In labelImg, make sure to select **"YOLO"** format
> (not Pascal VOC) in the left sidebar before saving.

---

## 8. Using the Jupyter Notebook

The Jupyter notebook provides an interactive, visual workflow.

### Start JupyterLab

```bash
cd ~/ROS_Workshop
uv run jupyter lab YoloExamples/yolo_training_workflow.ipynb
```

### What the Notebook Includes

| Step | Cell | What it does |
|------|------|-------------|
| 0 | Prerequisites | Verify ultralytics is installed |
| 1 | Define Classes | Set your class names |
| 2 | Create Structure | Auto-create dataset folders + data.yaml |
| 3 | Capture Images | Capture from webcam (optional) |
| 4 | Annotate | Inline OpenCV annotator |
| 5 | Auto-Split | Split images 80/20 into train/val |
| 6 | Verify | Count images/labels, show samples with boxes |
| 7 | Train | Run training with progress output |
| 8 | Evaluate | Show training curves, confusion matrix, metrics |
| 9 | Test | Predict on new images or webcam |
| 10 | Export | Convert to ONNX, TFLite, etc. |

### Advantages of the Notebook

- **Visual feedback** — See annotated images inline with matplotlib
- **Step-by-step** — Run one cell at a time, check results
- **Training curves** — See loss and mAP plots as training progresses
- **Easy experimentation** — Change hyperparameters and re-run

---

## 9. Training — How It Works Under the Hood

### What is Transfer Learning?

Instead of training from scratch (needs millions of images), we start
from `yolov8n.pt` which already knows general visual features:

```
Pre-trained yolov8n.pt knows:
  Layer 1-3:  Edges, corners, gradients
  Layer 4-6:  Textures, patterns, simple shapes
  Layer 7-10: Complex shapes, object parts
  Layer 11+:  Full objects (person, car, dog, etc.)

Transfer learning:
  ✓ Keep layers 1-10 (general features — still useful)
  ✗ Replace layer 11+ (retrain for YOUR specific objects)

Result: Your model learns YOUR objects using only 50-500 images
        instead of millions!
```

### The Training Loop

Each **epoch** (training round) does this:

```
For each epoch (1 to 50):
  │
  ├── Training Phase:
  │   For each batch of images:
  │     1. Feed images through the model
  │     2. Model predicts bounding boxes
  │     3. Compare predictions to YOUR labels (ground truth)
  │     4. Calculate "loss" (how wrong the model was)
  │     5. Adjust model weights to reduce the loss
  │        (this is called "backpropagation")
  │
  ├── Validation Phase:
  │   For each validation image:
  │     1. Feed image through the model (no weight updates)
  │     2. Compare predictions to labels
  │     3. Calculate mAP (accuracy metric)
  │
  └── Save checkpoint:
      If this epoch's mAP is the best so far → save as best.pt
      Always save as last.pt
```

### The Loss Functions

YOLO optimizes three losses simultaneously:

| Loss | What it measures | Goal |
|------|-----------------|------|
| **Box loss** | How accurate are the bounding box coordinates? | Tight boxes around objects |
| **Class loss** | How correct are the class predictions? | Right label for each object |
| **DFL loss** | Distribution focal loss for box regression | Better box edge predictions |

During training, you'll see these values decrease — that means the
model is learning!

```
Epoch  box_loss  cls_loss  dfl_loss  mAP50
1      2.45      3.12      1.89      0.15
10     1.23      1.45      1.12      0.55
25     0.78      0.89      0.92      0.78
50     0.52      0.56      0.81      0.89  ← Getting better!
```

### Data Augmentation

To make the model robust, YOLO automatically augments training images:

```
Original Image → Random transformations:

┌──────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐
│          │   │  Flipped  │   │ Brighter │   │  Mosaic  │
│  helmet  │   │  helmet   │   │  helmet  │   │ 4 images │
│          │   │          │   │          │   │ combined │
└──────────┘   └──────────┘   └──────────┘   └──────────┘

Augmentations applied:
  - Horizontal flip (50% chance)
  - Hue/saturation/brightness shifts
  - Mosaic (combines 4 images into one)
  - Scale and translation
```

This means even with 100 images, the model sees thousands of
variations — making it much more robust.

---

## 10. Training Commands — Step by Step

### Method 1: Using Our Training Script

```bash
# Step 1: Set up dataset structure
uv run python YoloExamples/train_custom_model.py \
    --setup --classes helmet no_helmet

# Step 2: Put images in the folders, then annotate
uv run python YoloExamples/annotate_images.py \
    --images YoloExamples/my_dataset/train/images/ \
    --classes helmet no_helmet

uv run python YoloExamples/annotate_images.py \
    --images YoloExamples/my_dataset/val/images/ \
    --classes helmet no_helmet

# Step 3: Train
uv run python YoloExamples/train_custom_model.py --train

# Step 4: Test on webcam
uv run python YoloExamples/train_custom_model.py \
    --predict --source 0

# Step 5: Test on an image
uv run python YoloExamples/train_custom_model.py \
    --predict --source path/to/test.jpg

# Step 6: Validate
uv run python YoloExamples/train_custom_model.py --validate

# Step 7: Export to ONNX
uv run python YoloExamples/train_custom_model.py --export onnx
```

### Method 2: Using the Ultralytics CLI

```bash
# Train
uv run yolo detect train \
    data=YoloExamples/my_dataset/data.yaml \
    model=yolov8n.pt \
    epochs=50 \
    imgsz=640 \
    batch=16 \
    name=helmet_model

# Predict on an image
uv run yolo detect predict \
    model=runs/detect/helmet_model/weights/best.pt \
    source=path/to/image.jpg \
    show=True

# Predict on webcam
uv run yolo detect predict \
    model=runs/detect/helmet_model/weights/best.pt \
    source=0 \
    show=True

# Validate
uv run yolo detect val \
    model=runs/detect/helmet_model/weights/best.pt \
    data=YoloExamples/my_dataset/data.yaml

# Export
uv run yolo export \
    model=runs/detect/helmet_model/weights/best.pt \
    format=onnx
```

### Method 3: Using Python Directly

```python
from ultralytics import YOLO

# Load pre-trained model
model = YOLO("yolov8n.pt")

# Train
model.train(
    data="YoloExamples/my_dataset/data.yaml",
    epochs=50,
    imgsz=640,
    batch=16,
    name="helmet_model",
)

# Validate
metrics = model.val()
print(f"mAP50: {metrics.box.map50:.4f}")

# Predict
results = model("test_image.jpg")
results[0].show()

# Export
model.export(format="onnx")
```

### Method 4: Using the Jupyter Notebook

```bash
uv run jupyter lab YoloExamples/yolo_training_workflow.ipynb
# Follow the cells step by step
```

### Training on a Downloaded Dataset

```bash
# Example: Using the Hard Hat Workers dataset from Roboflow
# After downloading and extracting:

uv run yolo detect train \
    data=YoloExamples/helmet_dataset/data.yaml \
    model=yolov8n.pt \
    epochs=50 \
    imgsz=640 \
    batch=16 \
    name=hardhat_model
```

---

## 11. Evaluating Your Model

### Understanding Training Output

After training, YOLO creates this folder:

```
runs/detect/helmet_model/
├── weights/
│   ├── best.pt              ← Best model (highest mAP) — USE THIS
│   └── last.pt              ← Last epoch model
├── results.png              ← Training curves (loss, mAP over epochs)
├── results.csv              ← Raw metrics per epoch
├── confusion_matrix.png     ← Which classes get confused
├── confusion_matrix_normalized.png
├── F1_curve.png             ← F1 score vs confidence threshold
├── P_curve.png              ← Precision vs confidence
├── R_curve.png              ← Recall vs confidence
├── PR_curve.png             ← Precision-Recall curve
├── labels.jpg               ← Distribution of labels in dataset
├── labels_correlogram.jpg   ← Box size/position distributions
├── train_batch0.jpg         ← Sample training batch with augmentation
├── val_batch0_labels.jpg    ← Validation ground truth
└── val_batch0_pred.jpg      ← Validation predictions
```

### Key Metrics Explained

| Metric | What it means | Good value |
|--------|--------------|------------|
| **mAP50** | Average precision at 50% IoU overlap | > 0.7 for a good model |
| **mAP50-95** | Average precision at 50-95% IoU (stricter) | > 0.5 |
| **Precision** | Of all detections, how many are correct? | > 0.8 |
| **Recall** | Of all real objects, how many were found? | > 0.8 |
| **F1 Score** | Balance between precision and recall | > 0.8 |

### Reading the Training Curves (results.png)

```
Loss curves (should go DOWN):
  train/box_loss  ↓  — Bounding box accuracy improving
  train/cls_loss  ↓  — Classification accuracy improving
  train/dfl_loss  ↓  — Distribution focal loss improving

Metric curves (should go UP):
  metrics/mAP50   ↑  — Overall detection accuracy improving
  metrics/mAP50-95 ↑ — Strict accuracy improving
  val/box_loss    ↓  — Validation loss (watch for overfitting)
```

### Signs of Overfitting

If training loss keeps going down but validation loss starts going UP:

```
         Training loss          Validation loss
Epoch 1:  ████████████  2.5      ████████████  2.5
Epoch 25: ████          1.0      ████          1.0  ← Both improving
Epoch 50: ██            0.5      ██████        1.5  ← OVERFITTING!
                                 ↑ Val loss going up = bad
```

**Fixes for overfitting:**
- Add more training images
- Use more data augmentation
- Train for fewer epochs
- Use a smaller model (nano instead of small)

---

## 12. Predicting with Your Trained Model

### On a Single Image

```bash
uv run yolo detect predict \
    model=runs/detect/helmet_model/weights/best.pt \
    source=test_image.jpg \
    conf=0.5 \
    show=True \
    save=True
# Saved result → runs/detect/predict/test_image.jpg
```

### On a Webcam

```bash
uv run yolo detect predict \
    model=runs/detect/helmet_model/weights/best.pt \
    source=0 \
    conf=0.5 \
    show=True
# Press 'q' to quit
```

### On a Video File

```bash
uv run yolo detect predict \
    model=runs/detect/helmet_model/weights/best.pt \
    source=video.mp4 \
    conf=0.5 \
    save=True
# Saved result → runs/detect/predict/video.mp4
```

### On a Folder of Images

```bash
uv run yolo detect predict \
    model=runs/detect/helmet_model/weights/best.pt \
    source=path/to/images/ \
    conf=0.5 \
    save=True
```

### In Python

```python
from ultralytics import YOLO
import cv2

model = YOLO("runs/detect/helmet_model/weights/best.pt")

# Single image
results = model("test.jpg", conf=0.5)
for box in results[0].boxes:
    cls = results[0].names[int(box.cls[0])]
    conf = float(box.conf[0])
    print(f"{cls}: {conf:.2f}")

# Webcam loop
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    results = model(frame, conf=0.5, verbose=False)
    cv2.imshow("Detection", results[0].plot())
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
```

---

## 13. Exporting for Deployment

### Export Formats

```bash
# ONNX — Universal, works everywhere
uv run yolo export model=best.pt format=onnx

# TensorRT — NVIDIA GPUs (fastest inference)
uv run yolo export model=best.pt format=engine

# TFLite — Android, Raspberry Pi, microcontrollers
uv run yolo export model=best.pt format=tflite

# OpenVINO — Intel CPUs/GPUs
uv run yolo export model=best.pt format=openvino

# CoreML — Apple devices (iOS, macOS)
uv run yolo export model=best.pt format=coreml
```

### Using an Exported Model

```python
# ONNX model
model = YOLO("best.onnx")
results = model("image.jpg")

# TensorRT model
model = YOLO("best.engine")
results = model("image.jpg")  # Much faster on NVIDIA GPU!
```

---

## 14. Complete Walkthrough: Helmet Detection

Here's the **entire process** from start to finish for a helmet
detection model using a public dataset.

### Step 1: Download the Dataset

```bash
# Go to: https://universe.roboflow.com/joseph-nelson/hard-hat-workers
# Click "Download Dataset" → Select "YOLOv8" → Download ZIP

# Or use a smaller helmet dataset:
# https://universe.roboflow.com/new-workspace-wz3ky/helmet-detection-yolov8
# Download in YOLOv8 format

# Extract to YoloExamples/
unzip helmet-detection.zip -d YoloExamples/helmet_dataset/
```

### Step 2: Verify the Dataset

```bash
# Check structure
ls YoloExamples/helmet_dataset/
# Expected: data.yaml  train/  valid/ (or val/)  test/

# Check data.yaml
cat YoloExamples/helmet_dataset/data.yaml
# Should show paths, nc (number of classes), and names

# Count images
ls YoloExamples/helmet_dataset/train/images/ | wc -l
ls YoloExamples/helmet_dataset/valid/images/ | wc -l
```

### Step 3: Train

```bash
uv run yolo detect train \
    data=YoloExamples/helmet_dataset/data.yaml \
    model=yolov8n.pt \
    epochs=50 \
    imgsz=640 \
    batch=16 \
    name=helmet_v1
```

**Expected output:**
```
Epoch  GPU_mem  box_loss  cls_loss  dfl_loss  Instances  Size
1/50   2.4G     2.456     3.123     1.892     45         640
...
50/50  2.4G     0.523     0.567     0.812     38         640

Results saved to runs/detect/helmet_v1
```

### Step 4: Check Results

```bash
# View training curves
# Open runs/detect/helmet_v1/results.png

# Validate
uv run yolo detect val \
    model=runs/detect/helmet_v1/weights/best.pt \
    data=YoloExamples/helmet_dataset/data.yaml
```

### Step 5: Test on Webcam

```bash
uv run yolo detect predict \
    model=runs/detect/helmet_v1/weights/best.pt \
    source=0 \
    show=True \
    conf=0.5
```

### Step 6: Connect to ESP8266

```python
# Detect helmet → LED green, No helmet → LED red (alarm)
from ultralytics import YOLO
import cv2
import socket

model = YOLO("runs/detect/helmet_v1/weights/best.pt")
cap = cv2.VideoCapture(0)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ESP_IP, ESP_PORT = "10.160.6.231", 4210

while True:
    ret, frame = cap.read()
    results = model(frame, conf=0.5, verbose=False)

    has_helmet = False
    has_no_helmet = False

    for box in results[0].boxes:
        cls_name = results[0].names[int(box.cls[0])]
        if "helmet" in cls_name.lower() and "no" not in cls_name.lower():
            has_helmet = True
        elif "no" in cls_name.lower() or "head" in cls_name.lower():
            has_no_helmet = True

    if has_no_helmet:
        sock.sendto(b"on", (ESP_IP, ESP_PORT))   # Alert!
    elif has_helmet:
        sock.sendto(b"off", (ESP_IP, ESP_PORT))  # All clear

    cv2.imshow("Helmet Check", results[0].plot())
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
```

---

## 15. Complete Walkthrough: Face Mask Detection

### Step 1: Download

```bash
# From Kaggle:
# https://www.kaggle.com/datasets/andrewmvd/face-mask-detection
# Download and extract

# Or from Roboflow (YOLO format, easier):
# https://universe.roboflow.com/pyimagesearch/face-mask-detection-wfkhi
# Download in YOLOv8 format
unzip face-mask-detection.zip -d YoloExamples/mask_dataset/
```

### Step 2: Train

```bash
uv run yolo detect train \
    data=YoloExamples/mask_dataset/data.yaml \
    model=yolov8n.pt \
    epochs=50 \
    imgsz=640 \
    name=mask_v1
```

### Step 3: Test

```bash
uv run yolo detect predict \
    model=runs/detect/mask_v1/weights/best.pt \
    source=0 \
    show=True
```

---

## 16. Tips for Better Models

### Dataset Quality

| Tip | Why |
|-----|-----|
| **More images** (100+) | More data = better generalization |
| **Diverse backgrounds** | Model learns the object, not the background |
| **Different lighting** | Works in bright and dark conditions |
| **Different angles** | Detects from any viewpoint |
| **Different distances** | Detects close-up and far away |
| **Include hard cases** | Partially hidden, blurry, small objects |
| **Balanced classes** | Similar number of images per class |
| **Accurate labels** | Tight boxes, no missing annotations |

### Training Hyperparameters

| Parameter | Default | When to change |
|-----------|---------|---------------|
| `epochs` | 50 | Increase to 100 if mAP is still improving |
| `batch` | 16 | Decrease to 8 or 4 if GPU memory error |
| `imgsz` | 640 | Increase to 1280 for small objects |
| `lr0` | 0.01 | Decrease to 0.001 if training is unstable |
| `model` | yolov8n.pt | Use yolov8s.pt for better accuracy |

### Common Mistakes

| Mistake | Fix |
|---------|-----|
| Not enough images | Collect at least 50 per class |
| Inconsistent labeling | Re-check annotations for consistency |
| Wrong class IDs | Verify data.yaml matches your labels |
| Images without labels | Every image needs a .txt file |
| Training too long | Stop when val loss starts increasing |
| Model too large for hardware | Use yolov8n (nano) for CPU |

---

## 17. Troubleshooting

### "No labels found"
```bash
# Check that label files exist and match image names
ls YoloExamples/my_dataset/train/labels/
# img001.txt should exist for img001.jpg

# Check label format (should be: class_id cx cy w h)
cat YoloExamples/my_dataset/train/labels/img001.txt
# Expected: 0 0.45 0.60 0.30 0.40
```

### "CUDA out of memory"
```bash
# Reduce batch size
uv run yolo detect train data=data.yaml model=yolov8n.pt batch=8
# Or even smaller
uv run yolo detect train data=data.yaml model=yolov8n.pt batch=4
```

### "mAP is 0.0 after training"
- Check that labels are in the correct format
- Check that class IDs in labels match data.yaml
- Check that images and labels have matching filenames
- Try training for more epochs

### "Model detects everything as one class"
- Your classes might be too similar visually
- Add more diverse training images
- Check for labeling errors (wrong class IDs)

### "Training is very slow"
- Use a GPU if available (NVIDIA + CUDA)
- Use the nano model (`yolov8n.pt`)
- Reduce image size: `imgsz=320`
- Reduce batch size if GPU memory is the bottleneck

### Converting Pascal VOC XML to YOLO Format

Some datasets (like the Kaggle face mask dataset) use XML format.
Convert with this script:

```python
import xml.etree.ElementTree as ET
import os

def voc_to_yolo(xml_path, classes, img_w, img_h):
    """Convert Pascal VOC XML to YOLO format."""
    tree = ET.parse(xml_path)
    root = tree.getroot()
    lines = []
    for obj in root.findall("object"):
        name = obj.find("name").text
        if name not in classes:
            continue
        class_id = classes.index(name)
        bbox = obj.find("bndbox")
        x1 = float(bbox.find("xmin").text)
        y1 = float(bbox.find("ymin").text)
        x2 = float(bbox.find("xmax").text)
        y2 = float(bbox.find("ymax").text)
        cx = ((x1 + x2) / 2) / img_w
        cy = ((y1 + y2) / 2) / img_h
        w = (x2 - x1) / img_w
        h = (y2 - y1) / img_h
        lines.append(f"{class_id} {cx:.6f} {cy:.6f} "
                     f"{w:.6f} {h:.6f}")
    return "\n".join(lines)
```

---

## 18. All Commands Quick Reference

### Setup

```bash
# Create dataset structure
uv run python YoloExamples/train_custom_model.py \
    --setup --classes helmet no_helmet

# Auto-split images 80/20
uv run python YoloExamples/train_custom_model.py \
    --split path/to/all_images/
```

### Annotate

```bash
# Our local tool
uv run python YoloExamples/annotate_images.py \
    --images path/to/images/ --classes helmet no_helmet

# labelImg
pip install labelImg && labelImg

# Jupyter notebook
uv run jupyter lab YoloExamples/yolo_training_workflow.ipynb
```

### Train

```bash
# Our script
uv run python YoloExamples/train_custom_model.py --train

# Ultralytics CLI
uv run yolo detect train data=data.yaml model=yolov8n.pt epochs=50

# Resume interrupted training
uv run python YoloExamples/train_custom_model.py --resume
```

### Predict

```bash
# Webcam
uv run yolo detect predict model=best.pt source=0 show=True

# Image
uv run yolo detect predict model=best.pt source=image.jpg show=True

# Video
uv run yolo detect predict model=best.pt source=video.mp4 save=True

# Folder of images
uv run yolo detect predict model=best.pt source=images/ save=True
```

### Validate

```bash
uv run yolo detect val model=best.pt data=data.yaml
```

### Export

```bash
uv run yolo export model=best.pt format=onnx
uv run yolo export model=best.pt format=tflite
uv run yolo export model=best.pt format=engine
```

---

## Further Reading

- **[Ultralytics Training Docs](https://docs.ultralytics.com/modes/train/)** — Official training guide
- **[Roboflow Universe](https://universe.roboflow.com)** — 250,000+ free datasets
- **[Kaggle Datasets](https://www.kaggle.com/datasets)** — Huge dataset collection
- **[YOLO Tips & Tricks](https://docs.ultralytics.com/guides/yolo-performance-metrics/)** — Performance optimization
- **[Data Augmentation Guide](https://docs.ultralytics.com/usage/cfg/#augmentation)** — All augmentation options
