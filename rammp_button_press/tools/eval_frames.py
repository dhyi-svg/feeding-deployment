"""Offline: run a detector over saved frames (real rgb+depth from ~/microwave_detections, plus
today's saved PNGs with a constant depth), optionally repainting the red panel cream."""
import glob, sys, json, importlib.util, numpy as np, cv2
S = str(__import__("pathlib").Path(__file__).resolve().parent)
K = np.load(f"{S}/frames/close24_K.npy"); FX = float(K[0])
def frames():
    for rgb in []:   # ~/microwave_detections depth PNGs are 8-bit visualisations, not depth: unusable
        dpath = rgb.replace("_rgb.jpg", "_depth.png")
        d = cv2.imread(dpath, cv2.IMREAD_UNCHANGED)
        if d is None: continue
        yield rgb.split("/")[-1][:-8], cv2.imread(rgb), d.astype(np.float32) / 1000.0
    for name, z in (("problem_view.png", 0.245), ("nofit.png", 0.48), ("staging_fits_raw.jpg", 0.245), ("staging_fits2_raw.jpg", 0.245)):
        img = cv2.imread(f"{S}/frames/{name}")
        if img is not None: yield name, img, np.full(img.shape[:2], z, np.float32)
def creamify(bgr, text_dark=True):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV); H, Sat, V = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    red = ((H < 10) | (H > 170)) & (Sat > 110) & (V > 35)
    out = bgr.astype(np.float32).copy()
    vr = np.median(V[red]) if red.any() else 150
    cream = np.array([190, 222, 240], np.float32)              # BGR, warm off-white
    scale = np.clip(V[red].astype(np.float32) / vr, 0.4, 1.15)[:, None]
    out[red] = np.clip(cream[None] * scale, 0, 255)
    if text_dark:  # white label text on the red panel -> dark text on cream (what a cream unit prints)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        near_red = cv2.dilate(red.astype(np.uint8), k) > 0
        text = near_red & ~red & (Sat < 80) & (V > 150)
        small = cv2.morphologyEx(text.astype(np.uint8), cv2.MORPH_OPEN, np.ones((1, 1), np.uint8)) > 0
        # only thin strokes (text), not the chrome domes: keep blobs whose area is small
        n, lab, st, _ = cv2.connectedComponentsWithStats(small.astype(np.uint8))
        for i in range(1, n):
            if st[i, 4] < 40: out[lab == i] = (60, 60, 70)
    return out.astype(np.uint8)
if __name__ == "__main__":
    mod = sys.argv[1]; mode = sys.argv[2] if len(sys.argv) > 2 else "red"
    spec = importlib.util.spec_from_file_location("dpx", mod); dpx = importlib.util.module_from_spec(spec); sys.modules["dpx"] = dpx; spec.loader.exec_module(dpx)
    res = {}
    for name, bgr, d in frames():
        img = creamify(bgr) if mode == "cream" else bgr
        f = dpx.detect(img, d, FX)
        res[name] = None if f is None else [round(v, 1) for v in f.px["start_30s"]] + [round(f.s_mm, 1)]
    print(json.dumps(res))
