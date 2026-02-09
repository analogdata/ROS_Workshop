"""
YOLO Image Annotation Tool — Web UI
====================================
A browser-based annotation tool for labeling images with bounding
boxes. Saves labels in YOLO format.

Branding: Analog Data AI (https://analogdata.ai)

Usage:
  uv run python YoloExamples/annotate/app.py

Then open http://localhost:5000 in your browser.
"""

from flask import (
    Flask, render_template, jsonify,
    request, send_file, Response, redirect, url_for
)
import os
import glob
import json
import io
import zipfile
import datetime

app = Flask(__name__)

# ─── Constants ────────────────────────────────────────────
IMAGE_EXTENSIONS = (
    "*.jpg", "*.jpeg", "*.png", "*.bmp", "*.webp",
    "*.JPG", "*.JPEG", "*.PNG",
)
UPLOAD_BASE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "_uploads"
)
METADATA_FILE = "annotation_meta.json"


# ─── Helpers ──────────────────────────────────────────────
def find_images(folder):
    imgs = []
    for ext in IMAGE_EXTENSIONS:
        imgs.extend(glob.glob(os.path.join(folder, ext)))
    return sorted(set(imgs))


def get_label_path(image_path, labels_dir):
    base = os.path.splitext(os.path.basename(image_path))[0]
    return os.path.join(labels_dir, base + ".txt")


def load_labels(image_path, labels_dir):
    lp = get_label_path(image_path, labels_dir)
    boxes = []
    if os.path.exists(lp):
        with open(lp, "r") as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) == 5:
                    boxes.append({
                        "class_id": int(parts[0]),
                        "cx": float(parts[1]),
                        "cy": float(parts[2]),
                        "w": float(parts[3]),
                        "h": float(parts[4]),
                    })
    return boxes


def save_labels_file(image_path, labels_dir, boxes):
    lp = get_label_path(image_path, labels_dir)
    os.makedirs(os.path.dirname(lp), exist_ok=True)
    with open(lp, "w") as f:
        for box in boxes:
            f.write(
                f"{box['class_id']} "
                f"{box['cx']:.6f} {box['cy']:.6f} "
                f"{box['w']:.6f} {box['h']:.6f}\n"
            )


def load_metadata(folder):
    mp = os.path.join(folder, METADATA_FILE)
    if os.path.exists(mp):
        with open(mp, "r") as f:
            return json.load(f)
    return {"classes": [], "created": "", "description": ""}


def save_metadata(folder, metadata):
    os.makedirs(folder, exist_ok=True)
    mp = os.path.join(folder, METADATA_FILE)
    with open(mp, "w") as f:
        json.dump(metadata, f, indent=2)
    cp = os.path.join(folder, "classes.txt")
    with open(cp, "w") as f:
        f.write("\n".join(metadata.get("classes", [])))


def resolve_labels_dir(images_dir):
    parent = os.path.dirname(images_dir)
    ld = os.path.join(parent, "labels")
    os.makedirs(ld, exist_ok=True)
    return ld


# ─── Page Routes ──────────────────────────────────────────
@app.route("/")
def home():
    return render_template("home.html", cwd=os.getcwd())


@app.route("/annotate")
def annotate_page():
    images_dir = request.args.get("dir", "")
    if not images_dir or not os.path.isdir(images_dir):
        return redirect(url_for("home"))
    labels_dir = resolve_labels_dir(images_dir)
    meta = load_metadata(labels_dir)
    classes = meta.get("classes", [])
    return render_template(
        "annotate.html",
        classes=classes,
        images_dir=images_dir,
    )


@app.route("/help")
def help_page():
    return render_template("help.html")


# ─── API: Folder browsing ────────────────────────────────
@app.route("/api/browse")
def api_browse():
    path = request.args.get("path", os.path.expanduser("~"))
    if not os.path.isdir(path):
        path = os.path.expanduser("~")
    entries = []
    try:
        for name in sorted(os.listdir(path)):
            full = os.path.join(path, name)
            if os.path.isdir(full) and not name.startswith("."):
                entries.append({"name": name, "path": full})
    except PermissionError:
        pass
    parent = os.path.dirname(path)
    img_count = len(find_images(path))
    return jsonify({
        "current": path,
        "parent": parent,
        "entries": entries,
        "has_images": img_count > 0,
        "image_count": img_count,
    })


# ─── API: Upload images ──────────────────────────────────
@app.route("/api/upload", methods=["POST"])
def api_upload():
    project = request.form.get("project", "my_project")
    project = "".join(
        c if c.isalnum() or c in "-_" else "_"
        for c in project
    )
    img_dir = os.path.join(UPLOAD_BASE, project, "images")
    lbl_dir = os.path.join(UPLOAD_BASE, project, "labels")
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(lbl_dir, exist_ok=True)
    files = request.files.getlist("files")
    count = 0
    for f in files:
        if f.filename:
            safe = f.filename.replace(" ", "_")
            f.save(os.path.join(img_dir, safe))
            count += 1
    return jsonify({"ok": True, "count": count, "dir": img_dir})


# ─── API: Images list ────────────────────────────────────
@app.route("/api/images")
def api_images():
    d = request.args.get("dir", "")
    if not d or not os.path.isdir(d):
        return jsonify({"images": [], "labeled": []})
    ld = resolve_labels_dir(d)
    imgs = find_images(d)
    data = []
    labeled = []
    for p in imgs:
        name = os.path.basename(p)
        data.append({"path": p, "name": name})
        lp = get_label_path(p, ld)
        if os.path.exists(lp) and os.path.getsize(lp) > 0:
            labeled.append(name)
    return jsonify({
        "images": data, "labeled": labeled, "folder": d,
    })


@app.route("/api/image/<int:idx>")
def api_image(idx):
    d = request.args.get("dir", "")
    imgs = find_images(d) if d and os.path.isdir(d) else []
    if idx < 0 or idx >= len(imgs):
        return "Not found", 404
    return send_file(imgs[idx])


# ─── API: Labels ─────────────────────────────────────────
@app.route("/api/labels/<int:idx>", methods=["GET"])
def api_get_labels(idx):
    d = request.args.get("dir", "")
    imgs = find_images(d) if d and os.path.isdir(d) else []
    if idx < 0 or idx >= len(imgs):
        return jsonify({"boxes": []})
    ld = resolve_labels_dir(d)
    return jsonify({"boxes": load_labels(imgs[idx], ld)})


@app.route("/api/labels/<int:idx>", methods=["POST"])
def api_save_labels(idx):
    d = request.args.get("dir", "")
    imgs = find_images(d) if d and os.path.isdir(d) else []
    if idx < 0 or idx >= len(imgs):
        return jsonify({"error": "Invalid"}), 400
    data = request.get_json()
    ld = resolve_labels_dir(d)
    save_labels_file(imgs[idx], ld, data.get("boxes", []))
    return jsonify({"ok": True})


# ─── API: Classes / Metadata ─────────────────────────────
@app.route("/api/classes", methods=["GET"])
def api_get_classes():
    d = request.args.get("dir", "")
    if not d:
        return jsonify({"classes": []})
    ld = resolve_labels_dir(d)
    meta = load_metadata(ld)
    return jsonify({"classes": meta.get("classes", [])})


@app.route("/api/classes", methods=["POST"])
def api_save_classes():
    d = request.args.get("dir", "")
    if not d:
        return jsonify({"error": "No dir"}), 400
    ld = resolve_labels_dir(d)
    data = request.get_json()
    meta = load_metadata(ld)
    meta["classes"] = data.get("classes", [])
    if not meta.get("created"):
        meta["created"] = datetime.datetime.now().isoformat()
    save_metadata(ld, meta)
    return jsonify({"ok": True})


# ─── API: Download ZIP ───────────────────────────────────
@app.route("/api/download_zip")
def api_download_zip():
    d = request.args.get("dir", "")
    imgs = find_images(d) if d and os.path.isdir(d) else []
    ld = resolve_labels_dir(d)
    meta = load_metadata(ld)
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as zf:
        zf.writestr(
            "classes.txt",
            "\n".join(meta.get("classes", []))
        )
        for p in imgs:
            lp = get_label_path(p, ld)
            if os.path.exists(lp):
                base = os.path.splitext(
                    os.path.basename(p)
                )[0]
                zf.write(lp, f"labels/{base}.txt")
    buf.seek(0)
    return Response(
        buf.getvalue(),
        mimetype="application/zip",
        headers={
            "Content-Disposition":
                "attachment; filename=yolo_labels.zip"
        },
    )


# ─── Main ─────────────────────────────────────────────────
if __name__ == "__main__":
    print()
    print("=" * 50)
    print("  YOLO Annotation Tool — Analog Data AI")
    print("=" * 50)
    print("  URL: http://localhost:5000")
    print("=" * 50)
    print()
    app.run(host="0.0.0.0", port=5000, debug=False)
