import cv2
import numpy as np
import matplotlib.pyplot as plt

def get_outer_corner(corners, idx, neigh_idx_1, neigh_idx_2):
    v_diff = corners[idx] - corners[neigh_idx_1]
    h_diff = corners[idx] - corners[neigh_idx_2]
    bottom_left = corners[idx] + v_diff + h_diff
    return bottom_left

def order_corners(pts):
    rect = np.zeros((4, 2), dtype="float32")

    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]  # top-left
    rect[3] = pts[np.argmax(s)]  # bottom-right
    
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]  # top-right
    rect[2] = pts[np.argmax(diff)]  # bottom-left
    return rect

def transform_image(img, corners, size=(800,800)):
    corners = np.float32(corners)
    corners = order_corners(corners)
    pts2 = np.float32([[0,0], [size[0], 0], [0, size[1]], [size[0], size[1]]])
    matrix = cv2.getPerspectiveTransform(corners, pts2)
    result = cv2.warpPerspective(img, matrix, size)
    return matrix, result

def find_chessbaord_transform(img, size=(800,800)):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    pattern_size = (7, 7)

    found, corners = cv2.findChessboardCorners(gray, pattern_size)
    if not found:
        print("Chessboard not found")
        return None

    corners = np.squeeze(corners)
    outer_corners = [
        get_outer_corner(corners, 0, 1, 7),
        get_outer_corner(corners, 6, 5, 13),
        get_outer_corner(corners, 42, 43, 35),
        get_outer_corner(corners, 48, 47, 41)
    ]
    print(outer_corners)

    corners = cv2.cornerSubPix(
        gray, corners, (5,5), (-1,-1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    )
    img_copy = img.copy()
    for corner in outer_corners:
        x, y = int(corner[0]), int(corner[1])
        cv2.circle(img_copy, (x, y), 10, (0, 0, 255), -1)

    for corner in corners:
        x, y = int(corner[0]), int(corner[1])
        cv2.circle(img_copy, (x, y), 5, (0, 255, 0), -1)
    
    plt.imshow(cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB))
    plt.axis("off")
    plt.show()

    return transform_image(img, outer_corners, size=size)

if __name__ == "__main__":
    img = cv2.imread("vision_system/img/image.png")

    mat, res = find_chessbaord_transform(img, size=(640, 640))
    cv2.imwrite("empty_proj.png", res)
    plt.imshow(cv2.cvtColor(res, cv2.COLOR_BGR2RGB))
    plt.axis("off")
    plt.show()


