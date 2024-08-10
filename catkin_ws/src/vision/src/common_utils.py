import numpy as np

# Given a bounding box and image, returns the image cropped 
# to the bounding box (to isolate detected objects).
def crop_to_bbox(image, bbox, copy=True):
    x_center, y_center, w, h = bbox
    x_min = x_center - w / 2
    y_min = y_center - h / 2
    x_min, x_max, y_min, y_max = int(x_min), int(x_min + w), int(y_min), int(y_min + h)
    if copy:
        return np.copy(image[y_min:y_max, x_min:x_max])
    else:
        return image[y_min:y_max, x_min:x_max]