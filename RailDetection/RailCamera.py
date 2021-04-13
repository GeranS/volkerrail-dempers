import cv2
import numpy as np
import Blob

TEST = False

# Set Values
PIXEL_PER_MM = 2.2
HALF_SCREEN = 320
old_trigger = 99

# PLC configuration
TCP_IP = '192.168.0.1'  # IP address of the PLC
TCP_PORT = 2000  # Port configured in the PLC

# Camera calibration property-strings
properties = ["CAP_PROP_FRAME_WIDTH",  # Width of the frames in the video stream.
              "CAP_PROP_FRAME_HEIGHT",  # Height of the frames in the video stream.
              "CAP_PROP_BRIGHTNESS",  # Brightness of the image (only for cameras).
              "CAP_PROP_CONTRAST",  # Contrast of the image (only for cameras).
              "CAP_PROP_SATURATION",  # Saturation of the image (only for cameras).
              "CAP_PROP_GAIN",  # Gain of the image (only for cameras).
              "CAP_PROP_EXPOSURE"]


def find_paste(image):
    """This function is a tool to distill the required blobs out of an image/frame"""

    # Get height and width from the grayscale image
    image_copy = image.copy()
    image_gray = cv2.cvtColor(image_copy, cv2.COLOR_BGR2GRAY)
    image_height, image_width = image_gray.shape

    # Cut away unnecessary parts of the image
    crop_away_top = int(image_height / 3) + 50
    crop_away_bottom = int(image_height / 3) - 50
    image_cropped = image_gray[crop_away_top: image_height - crop_away_bottom, 0: image_width]

    # Convert to a binary threshold image
    image_cropped_black_and_white = cv2.threshold(image_cropped, 120, 255, cv2.THRESH_BINARY)[1]

    # Delete noise from image
    kernel = np.ones((3, 5), np.uint8)
    erosion = cv2.erode(image_cropped_black_and_white, kernel, iterations=4)
    kernel = np.ones((9, 3), np.uint8)
    dilation = cv2.dilate(erosion, kernel, iterations=10)

    # Find the contour of denoised image and draw it
    contours, _ = cv2.findContours(dilation.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    output_image = cv2.cvtColor(dilation, cv2.COLOR_GRAY2BGR)

    # Interpret the contours
    for contour in contours:
        cv2.drawContours(output_image, [contour], 0, (0, 0, 255), 2)

        # Select only big contours
        if cv2.contourArea(contour) > 55000:

            # Draw a bounding rectangle and retreive position information
            x, y, w, h = cv2.boundingRect(contour)
            centre_x = int(x + (w / 2))
            centre_y = int(y + (h / 2))

            # Create a paste object from contour
            if RC.paste is None:
                RC.paste = Blob.Blob(h, w, x, y, centre_x, centre_y)

            # Update object
            elif abs(centre_x - RC.paste.centre_x) < 10:
                RC.paste.x = x
                RC.paste.y = y
                RC.paste.centre_x = centre_x
                RC.paste.centre_y = centre_y
                RC.paste.width = w
                RC.paste.height = h

                # Draw red rectangle
                if RC.paste.centre_x < HALF_SCREEN:
                    RC.paste.state = True
                    cv2.rectangle(image_copy, (RC.paste.x, RC.paste.y + crop_away_top),
                                  (RC.paste.x + RC.paste.width, RC.paste.y + RC.paste.height + crop_away_top),
                                  (0, 0, 255), 3)
                    cv2.circle(image_copy, (RC.paste.centre_x, RC.paste.centre_y + crop_away_top), 5, (0, 0, 255), 3)

                # Draw blue rectangle
                else:
                    cv2.rectangle(image_copy, (RC.paste.x, RC.paste.y + crop_away_top),
                                  (RC.paste.x + RC.paste.width, RC.paste.y + RC.paste.height + crop_away_top),
                                  (255, 0, 0), 3)
                    cv2.circle(image_copy, (RC.paste.centre_x, RC.paste.centre_y + crop_away_top), 5, (255, 0, 0), 3)

            # If the object replaces to quick the object will be None
            elif abs(centre_x - RC.paste.centre_x) > 10:
                RC.paste = None
    # No contours means No paste
    if len(contours) is None:
        RC.paste = None

    return output_image, image_copy, image_cropped_black_and_white


class RailCamera:
    def __init__(self):

        print("Initialising rail camera.")
        self.paste = None                                                   # Paste object
        self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)                       # Get camera

        self.gain = 0                                                       # Gain of the camera
        self.cap.set(cv2.CAP_PROP_GAIN, self.gain)                          # Apply defined gain to camera object
        self.brightness = 123                                               # Brightness of the camera
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.brightness)              # Apply defined brightness to camera object
        self.contrast = 85                                                  # Contrast of the camera
        self.cap.set(cv2.CAP_PROP_CONTRAST, self.contrast)                  # Apply defined contrast to camera object
        self.saturation = 100                                               # Saturation of the camera
        self.cap.set(cv2.CAP_PROP_SATURATION, self.saturation)              # Apply defined saturation to camera object
        self.exposure = -6                                                  # Exposure of the camera
        self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)                  # Apply defined exposure to camera object

        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))      # Get frame width
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))    # Get frame height

        print("brightness = {}".format(self.brightness))
        print("contrast = {}".format(self.contrast))
        print("saturation = {}".format(self.saturation))
        print("exposure = {}".format(self.exposure))
        print("frame width = {}".format(self.frame_width))
        print("frame height = {}".format(self.frame_height))

    # Method to tweek camera properties when TEST == True
    def set_cap_properties(self, key):
        print("\n\n")
        for prop in properties:
            val = self.cap.get(eval("cv2." + prop))
            print(prop + ": " + str(val))

        if key == ord('w'):
            self.brightness += 1
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.brightness)
        elif key == ord('s'):
            self.brightness -= 1
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.brightness)
        elif key == ord('d'):
            self.contrast += 1
            self.cap.set(cv2.CAP_PROP_CONTRAST, self.contrast)
        elif key == ord('a'):
            self.contrast -= 1
            self.cap.set(cv2.CAP_PROP_CONTRAST, self.contrast)
        elif key == ord('e'):
            self.saturation += 1
            self.cap.set(cv2.CAP_PROP_SATURATION, self.saturation)
        elif key == ord('q'):
            self.saturation -= 1
            self.cap.set(cv2.CAP_PROP_SATURATION, self.saturation)
        elif key == ord('z'):
            self.exposure += 1
            self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
        elif key == ord('c'):
            self.exposure -= 1
            self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)


if __name__ == "__main__":

    RC = RailCamera()
    trigger = 0

    while True:
        ret, RC.frame = RC.cap.read()

        # Display the resulting frame
        output_image, copy, threshold = find_paste(RC.frame)
        cv2.line(copy, (HALF_SCREEN, 0), (HALF_SCREEN, RC.frame_height), (0, 255, 0), 3)
        cv2.imshow('threshold', threshold)
        cv2.imshow('rail', output_image)
        cv2.imshow('normal', copy)

        if RC.paste is not None:
            if RC.paste.state:

                # Send trigger to PLC when the paste passes the camera
                if old_trigger != 3:
                    trigger = 3
                    old_trigger = 3
                    print("Trigger = {}".format(trigger))

        else:
            # Send reset trigger to PLC
            if old_trigger == 3:
                trigger = 99
                old_trigger = 99
                print("Trigger = {}".format(trigger))

        key = cv2.waitKey(1)

        if TEST:
            RC.set_cap_properties(key)

        if key == ord('k'):
            break

    RC.cap.release()
    cv2.destroyAllWindows()
