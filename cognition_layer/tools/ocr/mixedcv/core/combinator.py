"""
IOU
==============================
This module provides utilities for handling bounding boxes, including functions
to calculate the Intersection over Union (IoU) of bounding boxes and to merge
overlapping boxes based on a specified IoU threshold.
"""
# ======================= CONSTANTS ============================
TupledBoundingBox = tuple[
    int, int, int, int
]  # Type definition for a bounding box represented as a tuple of four integers.


# ======================= FUNCTIONS ============================
def iou(boxA, boxB):
    """
    - Calculate the Intersection over Union (IoU) between two bounding boxes.
    - :params:
        - boxA: The first bounding box.
        - boxB: The second bounding box.
    - :return: The IoU value as a float.
    """
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[0] + boxA[2], boxB[0] + boxB[2])
    yB = min(boxA[1] + boxA[3], boxB[1] + boxB[3])

    interArea = max(0, xB - xA) * max(0, yB - yA)
    boxAArea = boxA[2] * boxA[3]
    boxBArea = boxB[2] * boxB[3]

    iou = interArea / float(boxAArea + boxBArea - interArea + 1e-6)
    return iou


def merge_boxes(boxes, iou_threshold=0.3):
    """
    - Merge overlapping bounding boxes based on the specified IoU threshold.
    - :params:
        - boxes: A list of bounding boxes to merge.
        - iou_threshold: The threshold for merging boxes (default is 0.3).
    - :return: A list of merged bounding boxes.
    """
    merged = []
    used = [False] * len(boxes)

    for i in range(len(boxes)):
        if used[i]:
            continue

        # Start a new group with the current box
        current_box = boxes[i]
        group = [current_box]
        used[i] = True

        # Find all boxes that overlap with the current box
        for j in range(i + 1, len(boxes)):
            if not used[j] and iou(current_box, boxes[j]) > iou_threshold:
                group.append(boxes[j])
                used[j] = True

        # Combine the boxes in the group
        if len(group) == 1:
            merged.append(group[0])
        else:
            xs = [b[0] for b in group]
            ys = [b[1] for b in group]
            ws = [b[0] + b[2] for b in group]
            hs = [b[1] + b[3] for b in group]

            x_min = min(xs)
            y_min = min(ys)
            x_max = max(ws)
            y_max = max(hs)

            merged.append((x_min, y_min, x_max - x_min, y_max - y_min))

    return merged
