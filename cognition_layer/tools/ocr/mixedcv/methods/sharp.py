"""
sharpen_image_module
==============================
This module provides utilities for image processing, specifically for
sharpening images using a convolution kernel. It utilizes OpenCV for
image manipulation and NumPy for numerical operations.
"""
import cv2
import numpy as np


def sharp_image(imagen, k: int = 4):
    """
    - Applies a sharpening filter to the input image using a convolution kernel.
    - :params imagen: The input image to be sharpened.
    - :params k: The intensity of the sharpening effect. Higher values result in
    stronger sharpening.
    - :return: The sharpened image.
    """
    kernel = np.array([[0, -1, 0], [-1, k, -1], [0, -1, 0]])
    sharpened = cv2.filter2D(imagen, -1, kernel)
    return sharpened
