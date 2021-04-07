import cv2
import numpy as np

TEST = False

PIXEL_PER_MM = 2.2
HALF_SCREEN = 320

properties = ["CAP_PROP_FRAME_WIDTH",  # Width of the frames in the video stream.
              "CAP_PROP_FRAME_HEIGHT",  # Height of the frames in the video stream.
              "CAP_PROP_BRIGHTNESS",  # Brightness of the image (only for cameras).
              "CAP_PROP_CONTRAST",  # Contrast of the image (only for cameras).
              "CAP_PROP_SATURATION",  # Saturation of the image (only for cameras).
              "CAP_PROP_GAIN",  # Gain of the image (only for cameras).
              "CAP_PROP_EXPOSURE"]

def halfway_trigger():


def find_paste(image):
    image_copy = image.copy()
    image_gray = cv2.cvtColor(image_copy, cv2.COLOR_BGR2GRAY)
    image_height, image_width = image_gray.shape

    crop_away_top = int(image_height / 3) + 50
    crop_away_bottom = int(image_height / 3) - 50
    image_cropped = image_gray[crop_away_top: image_height - crop_away_bottom, 0: image_width]
    image_cropped_black_and_white = cv2.threshold(image_cropped, 120, 255, cv2.THRESH_BINARY)[1]

    kernel = np.ones((3, 5), np.uint8)
    erosion = cv2.erode(image_cropped_black_and_white, kernel, iterations=4)
    kernel = np.ones((9, 3), np.uint8)
    dilation = cv2.dilate(erosion, kernel, iterations=10)

    contours, _ = cv2.findContours(dilation.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    output_image = cv2.cvtColor(dilation, cv2.COLOR_GRAY2BGR)

    for contour in contours:
        cv2.drawContours(output_image, [contour], 0, (0, 0, 255), 2)

        x, y, w, h = cv2.boundingRect(contour)
        centre_x = int(x + (w / 2))
        centre_y = int(y + (h / 2))

        cv2.rectangle(output_image, (x, y), (x + w, y + h), (255, 0, 0), 5)
        cv2.circle(output_image, (centre_x, centre_y), 5, (255, 0, 0), 5)

        cv2.rectangle(image_copy, (x, y + crop_away_top), (x + w, y + h + crop_away_top), (255, 0, 0), 5)
        cv2.circle(image_copy, (centre_x, centre_y + crop_away_top), 5, (255, 0, 0), 5)

        if TEST:
            perimeter = cv2.arcLength(contour, True)
            print("Perimeter: {}".format(perimeter))
            print("Width = {:.1f} ; Height = {:.1f}".format(w / PIXEL_PER_MM, h / PIXEL_PER_MM))

    return output_image, image_copy, image_cropped_black_and_white


class RailCamera:
    def __init__(self):
        # todo: determine what kind of camera to use and do the setup here, likely to just be a normal camera but
        #  we're not sure yet
        print("Initialising rail camera.")
        self.user_input = input("Photo (p) or camera (c)")
        self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

        self.brightness = 123
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.brightness)
        self.contrast = 85
        self.cap.set(cv2.CAP_PROP_CONTRAST, self.contrast)
        self.saturation = 100
        self.cap.set(cv2.CAP_PROP_SATURATION, self.saturation)
        self.exposure = -6
        self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)

        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        print("brightness = {}".format(self.brightness))
        print("contrast = {}".format(self.contrast))
        print("saturation = {}".format(self.saturation))
        print("exposure = {}".format(self.exposure))
        print("frame width = {}".format(self.frame_width))
        print("frame height = {}".format(self.frame_height))

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
    # if TEST:
    #     output_image, threshold, erosion = find_paste(RC.frame)
    # else:
    RC = RailCamera()

    while True:
        ret, RC.frame = RC.cap.read()

        # Display the resulting frame
        output_image, copy, threshold = find_paste(RC.frame)
        cv2.line(copy, (HALF_SCREEN, 0), (HALF_SCREEN, RC.frame_height), (0, 255, 0), 3)
        cv2.imshow('threshold', threshold)
        cv2.imshow('rail', output_image)
        cv2.imshow('normal', copy)

        key = cv2.waitKey(500)

        if TEST:
            RC.set_cap_properties(key)

        if key == ord('k'):
            break

    RC.cap.release()
    cv2.destroyAllWindows()
