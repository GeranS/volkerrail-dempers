import cv2
import numpy as np


def find_paste(image):
    image_copy = image.copy()

    image_height, image_width = image_copy.shape

    height_to_crop_away = int(image_height / 3)

    image_cropped = image_copy[height_to_crop_away: image_height - height_to_crop_away, 0: image_width]

    image_cropped_black_and_white = cv2.threshold(image_cropped, 128, 255, cv2.THRESH_BINARY)[1]

    iterations = 10
    kernel = np.ones((6, 6), np.uint8)
    erosion = cv2.erode(image_cropped_black_and_white, kernel, iterations=iterations)
    kernel = np.ones((5, 5), np.uint8)
    dilation = cv2.dilate(erosion, kernel, iterations=iterations)

    edged = cv2.Canny(dilation, 30, 200)

    contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    output_image = cv2.cvtColor(image_cropped_black_and_white, cv2.COLOR_GRAY2BGR)

    for contour in contours:
        cv2.drawContours(output_image, [contour], 0, (0, 0, 255), 2)

        x, y, w, h = cv2.boundingRect(contour)

        cv2.rectangle(output_image, (x, y), (x + w, y + h), (255, 0, 0), 5)
        cv2.circle(output_image, (int(x + (w / 2)), int(y + (h / 2))), 5, (255, 0, 0), 5)

    return output_image


class RailCamera:
    def __init__(self):
        # todo: determine what kind of camera to use and do the setup here, likely to just be a normal camera but
        #  we're not sure yet
        print("Initialising rail camera.")

        self.cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
        ret, self.frame = self.cap.read()
        self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)


if __name__ == "__main__":
    RC = RailCamera()
    output_image = find_paste(RC.frame)

    while True:
        cv2.imshow('rail', output_image)

        key = cv2.waitKey(1)

        if key == ord('k'):
            break
    RC.cap.release()
    cv2.destroyAllWindows()
