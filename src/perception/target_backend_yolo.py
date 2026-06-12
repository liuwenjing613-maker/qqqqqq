#!/usr/bin/env python3

def extract_yolo_target(
    msg,
    target_classes=None,
    image_width=640,
    image_height=480,
    min_score=0.10,
):
    """
    将 ai_msgs/msg/PerceptionTargets 转为统一 target dict。

    target_classes:
      None 或 [] 表示不过滤类别；
      ["cup", "bottle"] 表示只接受这些类别。

    返回:
      {
        "visible": True/False,
        "class_name": str,
        "score": float,
        "bbox": [x, y, w, h],
        "cx": float,
        "cy": float,
        "area_ratio": float
      }
    """
    if target_classes is None:
        target_classes = []

    target_classes = [str(x).strip() for x in target_classes if str(x).strip()]
    candidates = []

    for target in msg.targets:
        class_name = str(target.type)

        if target_classes and class_name not in target_classes:
            continue

        for roi in target.rois:
            score = float(roi.confidence)

            if score < min_score:
                continue

            rect = roi.rect
            x = int(rect.x_offset)
            y = int(rect.y_offset)
            w = int(rect.width)
            h = int(rect.height)

            if w <= 0 or h <= 0:
                continue

            cx = x + w / 2.0
            cy = y + h / 2.0
            area_ratio = (w * h) / float(image_width * image_height)

            candidates.append({
                "visible": True,
                "class_name": class_name,
                "score": score,
                "bbox": [x, y, w, h],
                "cx": float(cx),
                "cy": float(cy),
                "area_ratio": float(area_ratio),
            })

    if not candidates:
        return {"visible": False}

    # 优先高置信度，其次大框
    candidates.sort(
        key=lambda item: (item["score"], item["area_ratio"]),
        reverse=True
    )

    return candidates[0]
