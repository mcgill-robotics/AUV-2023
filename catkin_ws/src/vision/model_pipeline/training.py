# Only use this file if all the folders and augmentations are set-up (including runs folder).
# This file is meant to avoid using the ugly and annoying jupyter notebook's output format.

import os
import shutil
from ultralytics import YOLO
import torch


is_front_camera_training = False  # Change it to False if training for down camera.
################################# OS PARAMETERS ##################################
os_name = os.name
# Windows = nt, [Linux, Apple] = posix.
os_path = "\\" if os_name == "nt" else "/"
##################################################################################
############################## TRAINING  PARAMETERS ##############################
if is_front_camera_training:
    model_save_filename = "best_AUV_sim_front_camera_model.pt"
else:
    model_save_filename = "best_AUV_sim_down_camera_model.pt"
model_name = "yolov8n.pt"
epoch_increments = 200
batch_size = -1  # Auto Mode (60% GPU Memory): Use batch=-1 to automatically
# adjust batch size for approximately 60% CUDA memory
# utilization.
workers = 2
cache = False
pretrained = True
imgsz = [480, 640]
hsv_h = 0.015
hsv_s = 0.3
hsv_v = 0.3
translate = 0.0
scale = 0.0
fliplr = 0.5
flipud = 0.5
mosaic = 0.1
copy_paste = 0.0
erasing = 0.0
crop_fraction = 0.1
degrees = 180
##################################################################################

if __name__ == "__main__":
    data_yaml_file_absolute_path = os.path.abspath("data.yaml")

    model = YOLO("yolov8n.pt")  # Load a pretrained model.

    # Start the training process.
    while True:
        try:
            model.train(
                data=data_yaml_file_absolute_path,
                epochs=epoch_increments,
                imgsz=imgsz,
                batch=batch_size,
                pretrained=pretrained,
                task="detect",
                cache=cache,
                workers=workers,
                hsv_h=hsv_h,
                hsv_s=hsv_s,
                hsv_v=hsv_v,
                translate=translate,
                scale=scale,
                fliplr=fliplr,
                flipud=flipud,
                mosaic=mosaic,
                copy_paste=copy_paste,
                erasing=erasing,
                crop_fraction=crop_fraction,
                degrees=degrees,
            )
            shutil.copyfile(
                f"runs{os_path}detect{os_path}train{os_path}weights{os_path}best.pt",
                model_save_filename,
            )
        except RuntimeError as e:
            print(f"Caught a RuntimeError: {e}.")
            break  # Break out of the loop if an error occurs to prevent infinite loop.
