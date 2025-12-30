#! /usr/bin/env python3

import cv2
import numpy as np
from typing import Tuple, Optional, List


def show_img(img, title='Image'):
    cv2.imshow(title, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def compute_boundary_mask(foreground_padded: np.ndarray) -> np.ndarray:
    """
    foreground_padded: bool mask with 1-pixel padding
    Boundary pixel = foreground pixel that has at least one 4-neighbor background.
    """
    up    = np.zeros_like(foreground_padded); up[1:, :]   = foreground_padded[:-1, :]
    down  = np.zeros_like(foreground_padded); down[:-1,:] = foreground_padded[1:, :]
    left  = np.zeros_like(foreground_padded); left[:, 1:] = foreground_padded[:, :-1]
    right = np.zeros_like(foreground_padded); right[:, :-1]= foreground_padded[:, 1:]

    interior_4 = up & down & left & right
    boundary = foreground_padded & (~interior_4)
    return boundary


# Contours are continuous points (along the boundary) having same color or intensity. In OpenCV, contours are extracted from binary images 
# background is white
# find connect(black)
def find_contours(binary_image) -> Tuple[np.ndarray, List[np.ndarray], Optional[np.ndarray]]:
    """
    Pure NumPy external contour tracing for binary images (8-connected).
    Returns list of contours; each contour is array of points (x, y) where x=col, y=row.
    """
    if binary_image.ndim != 2:
        raise ValueError("binary_image must be a 2D array")

    # show_img(binary_image, 'Binary Image')
    # Foreground mask: True where pixel is non-zero
    foreground = (binary_image != 0)
    # show_img(foreground.astype(np.uint8)*255, 'Foreground Mask')

    # Pad by 1 pixel to avoid bounds checks when looking at neighbors
    foreground_padded = np.pad(foreground, 1, mode="constant", constant_values=False)

    # Identify boundary pixels of blobs
    boundary_padded = compute_boundary_mask(foreground_padded)
    show_img(boundary_padded.astype(np.uint8)*255, 'Boundary Mask')

    # Track which boundary pixels have been assigned to a contour already
    visited_boundary = np.zeros_like(boundary_padded, dtype=bool)

    # 8-connected neighbor offsets in CLOCKWISE order starting from East (right)
    # Each item is (delta_row, delta_col)
    neighbor_offsets_clockwise: List[Tuple[int, int]] = [
        (0, 1),    # East
        (1, 1),    # South-East
        (1, 0),    # South
        (1, -1),   # South-West
        (0, -1),   # West
        (-1, -1),  # North-West
        (-1, 0),   # North
        (-1, 1),   # North-East
    ]

    padded_height, padded_width = boundary_padded.shape
    contours: List[np.ndarray] = []

    def trace_single_contour(start_row: int, start_col: int) -> List[Tuple[int, int]]:
        """
        Trace one contour using a right-hand-rule style border following:
        - We stand on a boundary pixel.
        - We search the 8 neighbors in clockwise order, starting a bit "to the right"
          of the direction we came from. This tends to follow the boundary consistently.
        Returns a list of (row, col) positions in PADDED coordinates.
        """
        contour_pixels_rc: List[Tuple[int, int]] = [(start_row, start_col)]
        visited_boundary[start_row, start_col] = True

        current_row, current_col = start_row, start_col

        # Index into neighbor_offsets_clockwise indicating the direction of the previous move.
        # Starting value doesn't matter much; East is a common default.
        prev_dir_index = 0  # 0 = East

        # Safety cap: prevents infinite loops in weird edge cases
        max_steps = int(padded_height * padded_width * 0.5)

        for _ in range(max_steps):
            # Right-hand rule: start checking neighbors from "one step right" of where we came from.
            # In an 8-neighborhood, "right" is approximately -2 steps (or +6) in clockwise indexing.
            search_start_dir_index = (prev_dir_index + 6) % 8

            found_next = False
            next_dir_index = None
            next_row = None
            next_col = None

            # Check all 8 neighbors in clockwise order starting from search_start_dir_index
            for offset_index in range(8):
                candidate_dir_index = (search_start_dir_index + offset_index) % 8
                delta_row, delta_col = neighbor_offsets_clockwise[candidate_dir_index]

                candidate_row = current_row + delta_row
                candidate_col = current_col + delta_col

                if boundary_padded[candidate_row, candidate_col]:
                    found_next = True
                    next_dir_index = candidate_dir_index
                    next_row, next_col = candidate_row, candidate_col
                    break

            if not found_next:
                # Dead end: boundary mask may be fragmented
                break

            # Step to the next boundary pixel
            current_row, current_col = next_row, next_col
            prev_dir_index = next_dir_index

            contour_pixels_rc.append((current_row, current_col))
            visited_boundary[current_row, current_col] = True

            # Stop if we loop back to the start (closed contour)
            if (current_row, current_col) == (start_row, start_col):
                break

        return contour_pixels_rc

    # Scan for unvisited boundary pixels; each starts a new contour trace
    for row in range(1, padded_height - 1):
        for col in range(1, padded_width - 1):
            if boundary_padded[row, col] and not visited_boundary[row, col]:
                contour_rc = trace_single_contour(row, col)

                # Convert PADDED (row,col) to ORIGINAL (x,y):
                # original_row = row - 1, original_col = col - 1
                contour_xy = np.array([(c - 1, r - 1) for (r, c) in contour_rc], dtype=np.int32)

                # Remove consecutive duplicates (optional cleanup)
                if len(contour_xy) >= 2:
                    keep = np.ones(len(contour_xy), dtype=bool)
                    keep[1:] = np.any(contour_xy[1:] != contour_xy[:-1], axis=1)
                    contour_xy = contour_xy[keep]

                contours.append(contour_xy)

    return contours

    
if __name__ == "__main__":
    # Tiny demo
    # img = np.zeros((10, 12), dtype=np.uint8)
    # img[2:8, 3:9] = 255
    
    # Load a sample image
    image = cv2.imread('img.png', cv2.IMREAD_GRAYSCALE)
    ret, img = cv2.threshold(image, 127, 255, 0)
    show_img(img, 'Input Image')

    contours = find_contours(img)
    print(f"Contours found: {len(contours)}")
    for i, contour in enumerate(contours):
        print(f"Contour {i} length = {len(contour)}")
        print(f"All points:\n{contour}")

    cv2_contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    print(f'Rico: cv2 contours: {cv2_contours}')

    # Compare the contours
    # cv2_contours is a tuple, extract first contour and reshape
    cv2_contour_0 = cv2_contours[0].squeeze()  # Remove the extra dimension: (N, 1, 2) -> (N, 2)
    custom_contour_0 = contours[0]
    
    print(f"\nCustom contour shape: {custom_contour_0.shape}")
    print(f"OpenCV contour shape: {cv2_contour_0.shape}")
    print(f"\nAre they equal? {np.array_equal(custom_contour_0, cv2_contour_0)}")
    
    if not np.array_equal(custom_contour_0, cv2_contour_0):
        print(f"\nDifferences:")
        print(f"Custom has {len(custom_contour_0)} points")
        print(f"OpenCV has {len(cv2_contour_0)} points")
    
    

    # # # Find contours in the image
    
    # show_img(thresh, 'Thresholded Image')
    # show_img(image, 'Image')

    # im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
