import cv2
import numpy as np
import os
from pathlib import Path
# You should replace these 3 lines with the output in calibration step
DIM=(1920,1080)
K=np.array([[823.0538285919985, 0.0, 946.5417318097574], [0.0, 823.7598602143755, 540.9001646720733], [0.0, 0.0, 1.0]])
D=np.array([[0.11815011169757952], [-0.4269057694117253], [3.813834322383702], [-8.364493809967435]])
def undistort(img_path):
    img = cv2.imread(img_path)
    if img is None:
        print(f"Skipping unreadable image: {img_path}")
        return

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    cv2.imshow("undistorted", undistorted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    script_dir = Path(__file__).resolve().parent
    img_dir = script_dir / "calibration" / "pictures"

    if not img_dir.is_dir():
        raise FileNotFoundError(
            f"Calibration image directory not found: {img_dir}. "
            "Update the img_dir path in this script."
        )

    for img_file in sorted(os.listdir(img_dir)):
        if img_file.lower().endswith((".png", ".jpg", ".jpeg")):
            undistort(str(img_dir / img_file))