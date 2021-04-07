import cv2
import numpy as np

TEST = False


def find_paste(image):
    image_copy = image.copy()
    image_gray = cv2.cvtColor(image_copy, cv2.COLOR_BGR2GRAY)
    image_height, image_width = image_gray.shape

    crop_away_top = int(image_height / 3) + 50
    crop_away_bottom = int(image_height / 3) - 50
    image_cropped = image_gray[crop_away_top: image_height - crop_away_bottom, 0: image_width]
    image_cropped_black_and_white = cv2.threshold(image_cropped, 125, 255, cv2.THRESH_BINARY)[1]

    kernel = np.ones((2, 4), np.uint8)
    erosion = cv2.erode(image_cropped_black_and_white, kernel, iterations=4)
    kernel = np.ones((10, 4), np.uint8)
    dilation = cv2.dilate(erosion, kernel, iterations=20)

    contours, _ = cv2.findContours(dilation.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    output_image = cv2.cvtColor(dilation, cv2.COLOR_GRAY2BGR)

    for contour in contours:
        cv2.drawContours(output_image, [contour], 0, (0, 0, 255), 2)

        x, y, w, h = cv2.boundingRect(contour)
        centre_x = int(x + (w / 2))
        centre_y = int(y + (h / 2))

        cv2.rectangle(output_image, (x, y), (x + w, y + h), (255, 0, 0), 5)
        cv2.circle(output_image, (centre_x, centre_y), 5, (255, 0, 0), 5)

        cv2.rectangle(image_copy, (x, y+crop_away_top), (x + w, y + h + crop_away_top), (255, 0, 0), 5)
        cv2.circle(image_copy, (centre_x, centre_y + crop_away_top), 5, (255, 0, 0), 5)

        perimeter = cv2.arcLength(contour, True)

        print("Perimeter: {}".format(perimeter))
        print("Coordinates are ({};{})".format(centre_x, centre_y))

    if TEST:
        return output_image, image_cropped_black_and_white, erosion
    else:
        return output_image, image_copy


class RailCamera:
    def __init__(self):
        # todo: determine what kind of camera to use and do the setup here, likely to just be a normal camera but
        #  we're not sure yet
        print("Initialising rail camera.")
        self.user_input = input("Photo (p) or camera (c)")


if __name__ == "__main__":
    # if TEST:
    #     output_image, threshold, erosion = find_paste(RC.frame)
    # else:
    RC = RailCamera()
    RC.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

    brightness = 255  # min: 0, max: 255, increment:5
    RC.cap.set(cv2.CAP_PROP_BRIGHTNESS, focus)

    while True:
        #if RC.user_input == 'c':
        ret, RC.frame = RC.cap.read()
        #elif RC.user_input == 'p':
        #    RC.frame = cv2.imread(r'C:\Users\smr.hhs.laptop\Downloads\Pasta 2.jpg')

        output_image, copy = find_paste(RC.frame)
        cv2.imshow('rail', output_image)
        cv2.imshow('normal', copy)
        key = cv2.waitKey(1)

        if key == ord('k'):
            break
    RC.cap.release()
    cv2.destroyAllWindows()
