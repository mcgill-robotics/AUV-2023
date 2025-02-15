import pyzed.sl as sl

# path to udev: /etc/udev
# reload udev commands
# sudo udevadm control --reload-rules
# sudo udevadm trigger


def main():
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = (
        sl.RESOLUTION.AUTO
    )  # Use HD720 opr HD1200 video mode, depending on camera type.
    init_params.camera_fps = 30  # Set fps at 30

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Camera Open : " + repr(err) + ". Exit program.")
        exit()

    # Capture 50 frames and stop
    i = 0
    image_left = sl.Mat()
    image_right = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()
    while i < 10:
        # Grab an image, a RuntimeParameters object must be given to grab()
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # A new image is available if grab() returns SUCCESS
            zed.retrieve_image(image_left, sl.VIEW.LEFT)
            zed.retrieve_image(image_right, sl.VIEW.RIGHT)

            image_left.write(f"test_left_cam{i}.jpg")
            image_right.write(f"test_right_cam{i}.jpg")
            # timestamp = zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)  # Get the timestamp at the time the image was captured
            # print("Image resolution: {0} x {1} || Image timestamp: {2}\n".format(image_left.get_width(), image_left.get_height(),
            #       timestamp.get_milliseconds()))
            i = i + 1

    # Close the camera
    zed.close()


if __name__ == "__main__":
    main()
