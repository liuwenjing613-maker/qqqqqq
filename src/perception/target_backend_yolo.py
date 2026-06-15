#!/usr/bin/env python3

import cv2
import numpy as np

# COCO 词库内与书包相关的类别；模型可能返回其中任意一个。
BACKPACK_CLASS_ALIASES = frozenset({
    "backpack",
    "handbag",
    "suitcase",
    "bag",
    "school bag",
    "red backpack",
})

# 同义词组：target 与 detection 落入同一组即视为匹配
BAG_CLASS_GROUPS = (
    BACKPACK_CLASS_ALIASES,
)

# 与 target_backend_red.py 保持一致的 HSV 红色范围
_LOWER_RED1 = np.array([0, 50, 35])
_UPPER_RED1 = np.array([18, 255, 255])
_LOWER_RED2 = np.array([165, 50, 35])
_UPPER_RED2 = np.array([180, 255, 255])


def _normalize_class_name(name):
    return str(name).strip().lower()


def parse_target_classes(value):
    """
    解析 target_classes 字符串。
    空字符串或 None 表示不过滤类别（返回 []，由 _matches_target_class 放行全部）。
    """
    if value is None:
        return []
    text = str(value).strip()
    if not text:
        return []
    return [part.strip() for part in text.split(",") if part.strip()]


def _class_group(name):
    normalized = _normalize_class_name(name)
    for group in BAG_CLASS_GROUPS:
        if normalized in group:
            return group
    return frozenset({normalized})


def _fuzzy_class_match(class_name, target_name):
    """同义词组 + 子串模糊匹配。"""
    det = _normalize_class_name(class_name)
    tgt = _normalize_class_name(target_name)
    if not det or not tgt:
        return False
    if det == tgt:
        return True
    if det in tgt or tgt in det:
        return True
    det_group = _class_group(det)
    tgt_group = _class_group(tgt)
    if det_group & tgt_group:
        return True
    return False


def _matches_target_class(class_name, target_classes):
    if not target_classes:
        return True

    normalized = _normalize_class_name(class_name)
    normalized_targets = {_normalize_class_name(x) for x in target_classes if str(x).strip()}

    if normalized in normalized_targets:
        return True

    for target in normalized_targets:
        if _fuzzy_class_match(normalized, target):
            return True

    return False


def compute_bbox_red_ratio(frame, bbox):
    """计算 bbox 内红色像素占比（0~1）。"""
    if frame is None:
        return 0.0

    h, w = frame.shape[:2]
    x, y, bw, bh = bbox
    x1 = max(0, int(x))
    y1 = max(0, int(y))
    x2 = min(w, int(x + bw))
    y2 = min(h, int(y + bh))
    if x2 <= x1 or y2 <= y1:
        return 0.0

    roi = frame[y1:y2, x1:x2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = (
        cv2.inRange(hsv, _LOWER_RED1, _UPPER_RED1)
        | cv2.inRange(hsv, _LOWER_RED2, _UPPER_RED2)
    )
    return float(np.count_nonzero(mask)) / float(mask.size)


def _resolve_det_coord_size(image_width, image_height, det_coord_width=640, det_coord_height=640):
    """
    BPU 检测框坐标在 YOLO-World 模型输入空间内，默认是 640x640。

    注意：hobot_yolo_world 日志中的模型输入为 [1, 640, 640, 3]。即使相机原图是
    1280x720，检测框仍按 640x640 的 letterbox/pad 输入坐标输出，而不是 640x360。
    之前按原图宽高比推断 det_coord_height=360，会把 y/h 放大 2 倍，导致框落到错误
    位置，进而让红色 HSV 二次验证失败。
    """
    if det_coord_width <= 0:
        det_coord_width = 640
    if det_coord_height <= 0:
        det_coord_height = det_coord_width
    return int(det_coord_width), int(det_coord_height)


def _normalize_raw_rect(x, y, w_field, h_field, coord_width=640, coord_height=640):
    """
    hobot_yolo_world 的 ROI 有时把 width/height 填成右下角 (x2,y2)，而非宽高。
    用几何约束 + 启发式区分两种格式，统一转为 (x, y, w, h)。
    """
    x = int(round(float(x)))
    y = int(round(float(y)))
    w_field = float(w_field)
    h_field = float(h_field)

    if w_field <= 0 or h_field <= 0:
        return x, y, 0, 0

    looks_like_x2y2 = (
        w_field > x
        and h_field > y
        and (
            (x + w_field > coord_width + 2)
            or (y + h_field > coord_height + 2)
            or ((w_field - x) < w_field * 0.6)
        )
    )
    if looks_like_x2y2:
        w = int(round(w_field - x))
        h = int(round(h_field - y))
    else:
        w = int(round(w_field))
        h = int(round(h_field))

    w = max(0, min(w, coord_width - max(0, x)))
    h = max(0, min(h, coord_height - max(0, y)))
    return x, y, w, h


def _bbox_iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    x1 = max(ax, bx)
    y1 = max(ay, by)
    x2 = min(ax + aw, bx + bw)
    y2 = min(ay + ah, by + bh)
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    if inter <= 0:
        return 0.0
    union = aw * ah + bw * bh - inter
    if union <= 0:
        return 0.0
    return inter / float(union)


def _confirm_red_target(frame, bbox, min_red_ratio=0.06, min_red_iou=0.10):
    """
    红色书包确认：优先看框内 HSV 红色占比；不足时用全图 HSV 红区与 YOLO 框 IoU 兜底。
    解决图像/检测不同步时框内 red_ratio=0 的误判。
    """
    ratio = compute_bbox_red_ratio(frame, bbox)
    if ratio >= min_red_ratio:
        return True, ratio

    from src.perception.target_backend_red import find_red_target

    red = find_red_target(frame)
    if not red.get("visible", False):
        return False, ratio

    iou = _bbox_iou(bbox, red["bbox"])
    if iou >= min_red_iou:
        return True, max(ratio, float(red.get("area_ratio", 0.0)))

    return False, ratio


def _pick_best_rejected(candidates, max_area_ratio):
    """失败时返回最有参考价值的小框，避免选超大误检框。"""
    reasonable = [
        c for c in candidates
        if c.get("reject_reason") != "area_too_large" and c["area_ratio"] <= max_area_ratio
    ]
    pool = reasonable or [
        c for c in candidates if c["area_ratio"] <= max_area_ratio * 1.05
    ]
    if not pool:
        return None
    return max(
        pool,
        key=lambda item: (item.get("red_ratio", 0.0), item["score"], -item["area_ratio"]),
    )


def _nms_candidates(candidates, iou_threshold=0.45):
    """合并高度重叠的重复框，保留分数更高者。"""
    if len(candidates) <= 1:
        return candidates

    ordered = sorted(
        candidates,
        key=lambda item: (item["score"], item.get("red_ratio", 0.0), item["area_ratio"]),
        reverse=True,
    )
    kept = []
    for cand in ordered:
        if all(_bbox_iou(cand["bbox"], k["bbox"]) < iou_threshold for k in kept):
            kept.append(cand)
    return kept


def _scale_bbox(x, y, w, h, image_width, image_height, det_coord_width=640, det_coord_height=640):
    cw, ch = _resolve_det_coord_size(image_width, image_height, det_coord_width, det_coord_height)
    sx = image_width / float(cw)
    sy = image_height / float(ch)
    return (
        int(round(x * sx)),
        int(round(y * sy)),
        max(1, int(round(w * sx))),
        max(1, int(round(h * sy))),
    )


def _bbox_plausibility(x, y, w, h, image_width, image_height):
    """评估 bbox 在原图像坐标系下是否合理（越高越好，<0 表示无效）。"""
    if w <= 0 or h <= 0 or image_width <= 0 or image_height <= 0:
        return -1.0

    x2 = x + w
    y2 = y + h
    vis_x1 = max(0, x)
    vis_y1 = max(0, y)
    vis_x2 = min(image_width, x2)
    vis_y2 = min(image_height, y2)
    vis_w = vis_x2 - vis_x1
    vis_h = vis_y2 - vis_y1
    if vis_w <= 0 or vis_h <= 0:
        return -1.0

    visible_ratio = (vis_w * vis_h) / float(w * h)
    area_ratio = (w * h) / float(image_width * image_height)
    if area_ratio > 0.55 or area_ratio < 0.0003:
        return -1.0

    off_x = max(0, -x) + max(0, x2 - image_width)
    off_y = max(0, -y) + max(0, y2 - image_height)
    off_penalty = (off_x / float(image_width) + off_y / float(image_height)) * 2.0

    # 偏好可见、适中面积（约 2%~20%）的框
    area_target = 0.08
    area_penalty = abs(area_ratio - area_target) * 1.5
    return visible_ratio - off_penalty - area_penalty


def _parse_roi_to_image_bbox(
    rect_x,
    rect_y,
    rect_w,
    rect_h,
    image_width,
    image_height,
):
    """
    将 PerceptionTargets roi.rect 转为原图像素 bbox [x,y,w,h]。

    hobot_yolo_world 可能输出 640 模型空间或 1280 原图空间；对多种
    (coord_canvas, normalize, scale) 组合打分，选最 plausible 的结果，
    避免 1280 原生坐标被二次缩放导致框偏到画面右侧。
    """
    configs = (
        (640, 640, True),
        (image_width, image_height, False),
    )
    best = None
    best_score = -1.0

    for coord_w, coord_h, do_scale in configs:
        nx, ny, nw, nh = _normalize_raw_rect(
            rect_x, rect_y, rect_w, rect_h, coord_w, coord_h
        )
        if nw <= 0 or nh <= 0:
            continue
        if do_scale:
            bx, by, bw, bh = _scale_bbox(
                nx, ny, nw, nh, image_width, image_height, coord_w, coord_h
            )
        else:
            bx, by, bw, bh = int(nx), int(ny), int(nw), int(nh)

        score = _bbox_plausibility(bx, by, bw, bh, image_width, image_height)
        if score > best_score:
            best_score = score
            best = (bx, by, bw, bh)

    if best is not None:
        return best

    nx, ny, nw, nh = _normalize_raw_rect(rect_x, rect_y, rect_w, rect_h, 640, 640)
    if nw <= 0 or nh <= 0:
        return 0, 0, 0, 0
    return _scale_bbox(nx, ny, nw, nh, image_width, image_height, 640, 640)


def _bbox_aspect(w, h):
    if h <= 0:
        return 0.0
    return w / float(h)


def _collect_yolo_candidates(
    msg,
    target_classes=None,
    image_width=640,
    image_height=480,
    min_score=0.002,
    max_area_ratio=0.15,
    min_area_ratio=0.002,
    max_aspect=4.0,
    min_aspect=0.20,
    frame=None,
    min_red_ratio=0.0,
    require_red_verify=False,
    min_red_iou=0.10,
    det_coord_width=640,
    det_coord_height=640,
):
    if target_classes is None:
        target_classes = []

    target_classes = [str(x).strip() for x in target_classes if str(x).strip()]
    candidates = []
    frame_area = float(image_width * image_height)

    for target in msg.targets:
        class_name = str(target.type)

        if not _matches_target_class(class_name, target_classes):
            continue

        for roi in target.rois:
            score = float(roi.confidence)
            if score < min_score:
                continue

            rect = roi.rect
            x, y, w, h = _parse_roi_to_image_bbox(
                rect.x_offset,
                rect.y_offset,
                rect.width,
                rect.height,
                image_width,
                image_height,
            )

            if w <= 0 or h <= 0:
                continue

            area_ratio = (w * h) / frame_area
            aspect = _bbox_aspect(w, h)
            red_ratio = 0.0
            red_ok = True
            if frame is not None:
                if require_red_verify:
                    red_ok, red_ratio = _confirm_red_target(
                        frame, [x, y, w, h], min_red_ratio, min_red_iou
                    )
                else:
                    red_ratio = compute_bbox_red_ratio(frame, [x, y, w, h])

            reject_reason = None
            if area_ratio > max_area_ratio:
                reject_reason = "area_too_large"
            elif w > image_width * 0.70 and x <= image_width * 0.05:
                reject_reason = "edge_wide_false_positive"
            elif area_ratio < min_area_ratio:
                reject_reason = "area_too_small"
            elif aspect > max_aspect or aspect < min_aspect:
                reject_reason = "bad_aspect"
            elif require_red_verify and frame is not None and not red_ok:
                reject_reason = "low_red_ratio"

            cx = x + w / 2.0
            cy = y + h / 2.0

            item = {
                "visible": reject_reason is None,
                "class_name": class_name,
                "score": score,
                "bbox": [x, y, w, h],
                "cx": float(cx),
                "cy": float(cy),
                "area_ratio": float(area_ratio),
                "aspect": float(aspect),
                "red_ratio": float(red_ratio),
                "reject_reason": reject_reason,
            }
            candidates.append(item)

    return _nms_candidates(candidates)


def extract_yolo_target(
    msg,
    target_classes=None,
    image_width=640,
    image_height=480,
    min_score=0.002,
    max_area_ratio=0.15,
    min_area_ratio=0.002,
    max_aspect=4.0,
    min_aspect=0.20,
    frame=None,
    min_red_ratio=0.06,
    require_red_verify=False,
    min_red_iou=0.10,
    det_coord_width=640,
    det_coord_height=640,
):
    """
    将 ai_msgs/msg/PerceptionTargets 转为统一 target dict。

    低置信度场景下配合：
      - max_area_ratio: 过滤覆盖半幅画面的误检框
      - frame + min_red_ratio: HSV 红色二次确认（找红色书包时启用）

    返回:
      {
        "visible": True/False,
        "class_name": str,
        "score": float,
        "bbox": [x, y, w, h],
        "cx": float,
        "cy": float,
        "area_ratio": float,
        "red_ratio": float (optional),
        "reason": str (when not visible)
      }
    """
    candidates = _collect_yolo_candidates(
        msg,
        target_classes=target_classes,
        image_width=image_width,
        image_height=image_height,
        min_score=min_score,
        max_area_ratio=max_area_ratio,
        min_area_ratio=min_area_ratio,
        max_aspect=max_aspect,
        min_aspect=min_aspect,
        frame=frame if not require_red_verify else None,
        min_red_ratio=min_red_ratio,
        require_red_verify=False,
        min_red_iou=min_red_iou,
        det_coord_width=det_coord_width,
        det_coord_height=det_coord_height,
    )

    geom_valid = [c for c in candidates if c["visible"]]

    if require_red_verify and frame is not None and geom_valid:
        matched = []
        for cand in geom_valid:
            red_ok, red_ratio = _confirm_red_target(
                frame, cand["bbox"], min_red_ratio, min_red_iou
            )
            if red_ok:
                item = dict(cand)
                item["red_ratio"] = red_ratio
                matched.append(item)
        geom_valid = matched

    valid = geom_valid
    if not valid:
        best_rejected = _pick_best_rejected(candidates, max_area_ratio)
        if best_rejected is None:
            return {"visible": False, "reason": "no_detection"}
        return {
            "visible": False,
            "reason": best_rejected.get("reject_reason", "filtered"),
            "class_name": best_rejected["class_name"],
            "score": best_rejected["score"],
            "bbox": best_rejected["bbox"],
            "area_ratio": best_rejected["area_ratio"],
            "red_ratio": best_rejected.get("red_ratio", 0.0),
        }

    def _mvp_rank_key(item):
        cls = _normalize_class_name(item["class_name"])
        cls_bonus = 1 if cls in {"backpack", "handbag"} else 0
        return (
            item.get("red_ratio", 0.0),
            cls_bonus,
            item["score"] / max(item["area_ratio"], 1e-6),
            -item["area_ratio"],
            item["score"],
        )

    # YOLO-World 对红色书包常同时输出低分 backpack 与超大 suitcase 误检；
    # 红色确认后仍优先小面积 backpack/handbag，抑制 score 接近时的泛化大框。
    valid.sort(key=_mvp_rank_key, reverse=True)

    best = valid[0]
    return {
        "visible": True,
        "class_name": best["class_name"],
        "score": best["score"],
        "bbox": best["bbox"],
        "cx": best["cx"],
        "cy": best["cy"],
        "area_ratio": best["area_ratio"],
        "red_ratio": best.get("red_ratio", 0.0),
    }


def list_all_yolo_detections(
    msg,
    image_width=1280,
    image_height=720,
    min_score=0.0,
    det_coord_width=640,
    det_coord_height=640,
):
    """
    列出 PerceptionTargets 中全部 ROI，不做 target_classes 过滤。
    供 yolo_world_bbox_preview 等调试脚本打印原始检测输出。
    """
    frame_area = float(image_width * image_height)
    results = []

    for target_index, target in enumerate(msg.targets):
        class_name = str(target.type)
        for roi_index, roi in enumerate(target.rois):
            score = float(roi.confidence)
            if score < min_score:
                continue

            rect = roi.rect
            norm640 = _normalize_raw_rect(
                rect.x_offset,
                rect.y_offset,
                rect.width,
                rect.height,
                640,
                640,
            )
            x, y, w, h = _parse_roi_to_image_bbox(
                rect.x_offset,
                rect.y_offset,
                rect.width,
                rect.height,
                image_width,
                image_height,
            )
            if w <= 0 or h <= 0:
                continue
            area_ratio = (w * h) / frame_area
            results.append({
                "target_index": target_index,
                "roi_index": roi_index,
                "class_name": class_name,
                "score": score,
                "bbox": [x, y, w, h],
                "cx": float(x + w / 2.0),
                "cy": float(y + h / 2.0),
                "area_ratio": float(area_ratio),
                "rect_raw": {
                    "x_offset": float(rect.x_offset),
                    "y_offset": float(rect.y_offset),
                    "width": float(rect.width),
                    "height": float(rect.height),
                },
                "rect_norm_640": list(norm640),
            })

    results.sort(key=lambda item: item["score"], reverse=True)
    return results


def list_yolo_candidates(
    msg,
    target_classes=None,
    image_width=640,
    image_height=480,
    min_score=0.0,
    frame=None,
    min_red_ratio=0.06,
    require_red_verify=False,
    max_area_ratio=0.15,
    min_area_ratio=0.002,
    det_coord_width=640,
    det_coord_height=640,
    min_red_iou=0.10,
):
    """列出所有候选框及过滤状态，供诊断预览使用。"""
    return _collect_yolo_candidates(
        msg,
        target_classes=target_classes,
        image_width=image_width,
        image_height=image_height,
        min_score=min_score,
        max_area_ratio=max_area_ratio,
        min_area_ratio=min_area_ratio,
        frame=frame,
        min_red_ratio=min_red_ratio,
        require_red_verify=require_red_verify,
        min_red_iou=min_red_iou,
        det_coord_width=det_coord_width,
        det_coord_height=det_coord_height,
    )
