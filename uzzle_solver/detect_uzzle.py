#!/usr/bin/env python3
import cv2
import numpy as np
import argparse
from collections import deque, Counter
from typing import Optional, Tuple, List
from solver import solve

ROWS, COLS = 3, 4

# Warped card output size
WARP_W, WARP_H = 900, 600

# Tile hue targets in OpenCV HSV (H in 0..179)
# NOTE: purple is closer to ~160-170 for your camera/lighting, so use 165.
HUE_TARGETS = {
    "red": 0,
    "orange": 15,
    "green": 60,
    "blue": 110,
    "purple": 165,
}

LABEL_TO_SHORT = {
    "red": "R",
    "orange": "O",
    "green": "G",
    "blue": "B",
    "purple": "P",
    "white": ".",
    "unknown": "?"
}

def circular_hue_distance(h: float, target: float) -> float:
    d = abs(h - target)
    return min(d, 180 - d)

def circular_mean_hue(hues: np.ndarray) -> float:
    # hues in [0..179]
    angles = (hues * 2.0) * (np.pi / 180.0)  # map 0..179 -> 0..358 degrees
    sin_mean = float(np.mean(np.sin(angles)))
    cos_mean = float(np.mean(np.cos(angles)))
    ang = np.arctan2(sin_mean, cos_mean)
    if ang < 0:
        ang += 2 * np.pi
    return float((ang * 180.0 / np.pi) / 2.0)

def order_points(pts: np.ndarray) -> np.ndarray:
    # pts: (4,2)
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]  # tl
    rect[2] = pts[np.argmax(s)]  # br
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]  # tr
    rect[3] = pts[np.argmax(diff)]  # bl
    return rect

# ---------- Card detection (robust: minAreaRect + extent) ----------

def find_card_box_minarearect(frame: np.ndarray,
                              min_contour_area: int = 20000,
                              extent_min: float = 0.65,
                              ar_min: float = 1.1,
                              ar_max: float = 2.6,
                              debug: bool = False) -> Optional[np.ndarray]:
    """
    Returns 4 points (boxPoints) of the best rectangle candidate, or None.
    More robust than approxPolyDP==4 for real-world camera shots.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    edges = cv2.Canny(gray, 50, 150)
    edges = cv2.dilate(edges, None, iterations=2)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    best_box = None
    best_score = 0.0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_contour_area:
            continue

        rect = cv2.minAreaRect(cnt)
        (cx, cy), (w, h), ang = rect
        if w < 50 or h < 50:
            continue

        rect_area = w * h
        if rect_area <= 1:
            continue

        extent = float(area / rect_area)  # how well contour fills the rect
        ar = float(max(w, h) / (min(w, h) + 1e-6))

        if extent < extent_min:
            continue
        if not (ar_min <= ar <= ar_max):
            continue

        # score favors large area + filled rectangle
        score = area * extent

        if score > best_score:
            best_score = score
            box = cv2.boxPoints(rect).astype(np.float32)
            best_box = box

    if debug and best_box is not None:
        dbg = frame.copy()
        cv2.polylines(dbg, [best_box.astype(np.int32)], True, (0, 255, 0), 3)
        cv2.imshow("CardBoxDebug", dbg)

    return best_box

def warp_card(frame: np.ndarray, box4: np.ndarray, out_w=WARP_W, out_h=WARP_H) -> np.ndarray:
    rect = order_points(box4)
    dst = np.array([[0, 0], [out_w - 1, 0], [out_w - 1, out_h - 1], [0, out_h - 1]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(rect, dst)
    return cv2.warpPerspective(frame, M, (out_w, out_h))

# ---------- Grid detection via blue-line morphology + projections ----------

def _peak_centers_from_projection(proj: np.ndarray, frac: float) -> List[int]:
    """
    Turn a 1D projection into peak centers by thresholding at frac*max and grouping.
    """
    if proj.size == 0:
        return []
    th = float(proj.max()) * float(frac)
    idx = np.where(proj > th)[0]
    if len(idx) == 0:
        return []

    groups = []
    start = idx[0]
    prev = idx[0]
    for i in idx[1:]:
        if i == prev + 1:
            prev = i
        else:
            groups.append((start, prev))
            start = i
            prev = i
    groups.append((start, prev))
    centers = [(a + b) // 2 for a, b in groups]
    return centers

def find_grid_boundaries_from_blue(warped_bgr: np.ndarray,
                                   debug: bool = False) -> Optional[Tuple[List[int], List[int], np.ndarray]]:
    """
    Uses the blue grid lines to find grid boundaries.
    Returns (xs, ys, warped_inner) where:
      xs length 5 (4 cols -> 5 boundaries), ys length 4 (3 rows -> 4 boundaries)
    """
    H, W = warped_bgr.shape[:2]

    # Crop a little to reduce card outer border influence
    mx = int(W * 0.03)
    my = int(H * 0.03)
    warped_inner = warped_bgr[my:H - my, mx:W - mx].copy()

    hsv = cv2.cvtColor(warped_inner, cv2.COLOR_BGR2HSV)
    Hc, Sc, Vc = cv2.split(hsv)

    # Blue mask for grid lines/border (tuned for your screenshot)
    blue_mask = ((Hc >= 95) & (Hc <= 135) & (Sc > 60) & (Vc > 40)).astype(np.uint8) * 255
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=1)

    # Extract vertical lines
    v_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, max(25, warped_inner.shape[0] // 12)))
    v_lines = cv2.erode(blue_mask, v_kernel, iterations=1)
    v_lines = cv2.dilate(v_lines, v_kernel, iterations=1)

    # Extract horizontal lines
    h_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (max(25, warped_inner.shape[1] // 12), 1))
    h_lines = cv2.erode(blue_mask, h_kernel, iterations=1)
    h_lines = cv2.dilate(h_lines, h_kernel, iterations=1)

    v_proj = v_lines.sum(axis=0).astype(np.float32)
    h_proj = h_lines.sum(axis=1).astype(np.float32)

    xs = _peak_centers_from_projection(v_proj, frac=0.35)
    ys = _peak_centers_from_projection(h_proj, frac=0.35)

    # We expect 5 vertical boundaries and 4 horizontal boundaries.
    # If too many, keep the most evenly spaced 5/4 by sorting and downsampling.
    def downsample(vals: List[int], n: int) -> Optional[List[int]]:
        if len(vals) < n:
            return None
        vals = sorted(vals)
        if len(vals) == n:
            return vals
        idxs = [int(round(i * (len(vals) - 1) / (n - 1))) for i in range(n)]
        out = [vals[i] for i in idxs]
        # ensure strictly increasing
        for i in range(1, len(out)):
            if out[i] <= out[i - 1]:
                out[i] = out[i - 1] + 1
        return out

    xs = downsample(xs, 5)
    ys = downsample(ys, 4)
    if xs is None or ys is None:
        if debug:
            cv2.imshow("BlueMask", blue_mask)
            cv2.imshow("VLines", v_lines)
            cv2.imshow("HLines", h_lines)
        return None

    if debug:
        dbg = warped_inner.copy()
        for x in xs:
            cv2.line(dbg, (x, 0), (x, dbg.shape[0] - 1), (0, 255, 0), 2)
        for y in ys:
            cv2.line(dbg, (0, y), (dbg.shape[1] - 1, y), (0, 255, 0), 2)
        cv2.imshow("WarpedInner+GridLines", dbg)
        cv2.imshow("BlueMask", blue_mask)

    return xs, ys, warped_inner

# ---------- Cell color classification (ignore icon by using only saturated pixels) ----------

def classify_cell(patch_bgr: np.ndarray,
                  sat_thresh: int = 50,
                  val_thresh: int = 40,
                  colored_frac_thresh: float = 0.03) -> Tuple[str, float]:
    """
    Key idea: the white icon dominates the center. So:
      - Keep only pixels with decent saturation (background color)
      - If too few colored pixels -> 'white' (empty)
      - Otherwise classify by mean hue
    Returns (label, confidence-ish).
    """
    hsv = cv2.cvtColor(patch_bgr, cv2.COLOR_BGR2HSV)
    Hc, Sc, Vc = cv2.split(hsv)
    Hf = Hc.astype(np.float32)
    Sf = Sc.astype(np.float32)
    Vf = Vc.astype(np.float32)

    mask = (Sf >= sat_thresh) & (Vf >= val_thresh)
    frac = float(mask.mean())
    if frac < colored_frac_thresh:
        return "white", 0.6

    mean_h = circular_mean_hue(Hf[mask])

    best_label = "unknown"
    best_dist = 999.0
    for lbl, tgt in HUE_TARGETS.items():
        d = circular_hue_distance(mean_h, float(tgt))
        if d < best_dist:
            best_dist = d
            best_label = lbl

    conf = float(max(0.0, 1.0 - (best_dist / 30.0)))
    return best_label, conf

def mode_label(hist: deque) -> str:
    if not hist:
        return "unknown"
    return Counter(hist).most_common(1)[0][0]

def format_grid(labels: List[List[str]]) -> str:
    return " | ".join(" ".join(LABEL_TO_SHORT.get(labels[r][c], "?") for c in range(COLS)) for r in range(ROWS))

# ---------- Main loop ----------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--debug", action="store_true", help="show debug windows")
    ap.add_argument("--smooth", type=int, default=5, help="frames of smoothing per cell")
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {args.camera}. Try --camera 1")

    debug = bool(args.debug)
    smooth_n = max(1, int(args.smooth))
    hist = [[deque(maxlen=smooth_n) for _ in range(COLS)] for _ in range(ROWS)]

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        overlay = frame.copy()
        labels = [["?" for _ in range(COLS)] for _ in range(ROWS)]

        # 1) find card
        box = find_card_box_minarearect(frame, debug=False)
        if box is not None:
            cv2.polylines(overlay, [box.astype(np.int32)], True, (0, 255, 0), 2)

            # 2) warp
            warped = warp_card(frame, box)

            # 3) grid boundaries from blue
            grid = find_grid_boundaries_from_blue(warped, debug=debug)
            if grid is not None:
                xs, ys, warped_inner = grid

                # 4) classify each cell
                for r in range(ROWS):
                    for c in range(COLS):
                        x0, x1 = xs[c], xs[c + 1]
                        y0, y1 = ys[r], ys[r + 1]

                        # Inset to avoid blue grid lines and borders
                        ix = int((x1 - x0) * 0.22)
                        iy = int((y1 - y0) * 0.22)
                        x0i, x1i = x0 + ix, x1 - ix
                        y0i, y1i = y0 + iy, y1 - iy
                        if x1i <= x0i or y1i <= y0i:
                            continue

                        patch = warped_inner[y0i:y1i, x0i:x1i]
                        lbl, conf = classify_cell(patch)

                        # optional: if confidence super low, mark unknown
                        if lbl != "white" and conf < 0.25:
                            lbl = "unknown"

                        hist[r][c].append(lbl)
                        labels[r][c] = mode_label(hist[r][c])

                # nice debug overlay on warped
                if debug:
                    dbg = warped_inner.copy()
                    for r in range(ROWS):
                        for c in range(COLS):
                            x0, x1 = xs[c], xs[c + 1]
                            y0, y1 = ys[r], ys[r + 1]
                            cv2.rectangle(dbg, (x0, y0), (x1, y1), (255, 255, 255), 1)
                            cv2.putText(dbg, LABEL_TO_SHORT.get(labels[r][c], "?"),
                                        (x0 + 10, y0 + 28),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2, cv2.LINE_AA)
                    cv2.imshow("WarpedInner+Labels", dbg)

        # UI text
        grid_str = format_grid(labels)
        found = ("?" not in grid_str)
        cv2.putText(overlay,
                    grid_str if found else "No checkerboard found",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 255, 0) if found else (0, 0, 255),
                    2,
                    cv2.LINE_AA)
        if found:
            #TODO Remember to remove
            print(f'Rico: found checker board')
            assignment = solve(labels=labels)
            if assignment is not None:
                #TODO Remember to remove
                print(f'Rico: found assignment')
                for r in range(ROWS):
                    for c in range(COLS):
                        val = assignment[r][c]
                        cv2.putText(overlay,
                                    str(val),
                                    (50 + c * 100, 100 + r * 100),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    1.5,
                                    (255, 0, 0),
                                    3,
                                    cv2.LINE_AA)

        cv2.putText(overlay, "q=quit  d=debug on/off", (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.imshow("Uzzle Detector", overlay)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("d"):
            debug = not debug
            if not debug:
                for win in ["CardBoxDebug", "WarpedInner+GridLines", "BlueMask", "WarpedInner+Labels"]:
                    try:
                        cv2.destroyWindow(win)
                    except:
                        pass

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
