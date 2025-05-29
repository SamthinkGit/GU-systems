"""
detect_contour_regions
==============================
This module provides functionality to detect contour regions in an image using
OpenCV. It processes an image to identify contours and returns bounding boxes
for the specified number of contours based on a sorting criterion.
"""
import cv2

BoundingBox = tuple[int, int, int, int]  # (x, y, width, height)


# ======================= FUNCTIONS ============================
def detect_contour_regions(
    image,
    num_contours: int = 40,
    canny_p1: int = 10,
    canny_p2: int = 500,
    blur: int = 5,
) -> list[BoundingBox]:
    """
    - Detects contour regions in a given image and returns bounding boxes.
    - :params:
        - image: The input image in which contours are to be detected.
        - num_contours: The number of contours to return, default is 40.
        - blur: Size of the Gaussian blur to apply, default is 5.
    - :return: A list of bounding boxes for the detected contours.
    - :warning: Ensure that the input image is in the correct format
      for contour detection.
    """
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    blur = max(3, blur | 1)  # asegurar que el blur sea impar y >= 3
    blurred = cv2.GaussianBlur(gray_image, (blur, blur), 0)
    edges = cv2.Canny(blurred, canny_p1, canny_p2)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    sortedContours = sorted(contours, key=cv2.contourArea, reverse=True)

    boxes: list[BoundingBox] = []
    for cnt in sortedContours[:num_contours]:
        x, y, w, h = cv2.boundingRect(cnt)
        boxes.append((x, y, w, h))

    return boxes
