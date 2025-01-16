import cv2
import numpy as np


def create_mask(hsv, lower, upper):
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    return mask


def find_contours(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours


def get_contour_center(contour):
    M = cv2.moments(contour)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return (cX, cY)
    return None


def runPipeline(image, llrobot):
    # Get isBlue flag from llrobot input
    isBlue = llrobot[0] > 0 if len(llrobot) > 0 else True

    # Blur the image to reduce noise
    blurred = cv2.GaussianBlur(image, (15, 15), 0)

    # Convert to HSV color space
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Define color ranges
    lower_blue = np.array([100, 150, 0])
    upper_blue = np.array([140, 255, 255])
    lower_red1 = np.array([0, 150, 0])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 150, 0])
    upper_red2 = np.array([180, 255, 255])
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])

    # Create masks
    yellow_mask = create_mask(hsv, lower_yellow, upper_yellow)
    if isBlue:
        color_mask = create_mask(hsv, lower_blue, upper_blue)
        color_name = "blue"
    else:
        mask1 = create_mask(hsv, lower_red1, upper_red1)
        mask2 = create_mask(hsv, lower_red2, upper_red2)
        color_mask = cv2.bitwise_or(mask1, mask2)
        color_name = "red"

    # Find contours
    yellow_contours = find_contours(yellow_mask)
    color_contours = find_contours(color_mask)

    # Combine contours
    all_contours = yellow_contours + color_contours

    if not all_contours:
        return np.array([]), image, [0, 0, 0, 0, 0]

    # Find image center
    height, width = image.shape[:2]
    center_x, center_y = width // 2, height // 2

    # Find contour closest to center
    closest_contour = min(all_contours, key=lambda c:
    abs(get_contour_center(c)[0] - center_x) +
    abs(get_contour_center(c)[1] - center_y) if get_contour_center(c) else float('inf'))

    # Get properties of the closest contour
    rect = cv2.minAreaRect(closest_contour)
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    # Get the width and height of the bounding box
    width, height = rect[1]

    # Calculate the angle based on the longest side of the bounding box
    angle = rect[2]
    # Adjust angle for vertical (height > width) and horizontal (width > height) cases
    if width > height:
        # If height > width, it's closer to vertical orientation
        angle = 90 - abs(angle)
    else:
        # If width > height, it's closer to horizontal orientation
        angle = abs(angle)

    # Normalize the angle to be between 0° and 180°
    if angle < 0:
        angle += 180
    elif angle > 180:
        angle -= 180

    min_angle = 7
    max_angle = 63
    range_size = max_angle - min_angle + 1
    wrapped_angle = (-(range_size / 90) * angle) + max_angle
    # Get center of the contour
    cx, cy = get_contour_center(closest_contour)

    # Determine color of the selected contour
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.drawContours(mask, [closest_contour], 0, 255, -1)
    if cv2.countNonZero(cv2.bitwise_and(mask, yellow_mask)) > cv2.countNonZero(
            cv2.bitwise_and(mask, color_mask)):
        selected_color = "yellow"
    else:
        selected_color = color_name

    # Draw the selected contour and its properties on the image
    cv2.drawContours(image, [box], 0, (0, 255, 0), 2)
    cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)
    cv2.putText(image, f"Angle: {wrapped_angle:.2f}°", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 255, 0), 2)
    cv2.putText(image, f"Color: {selected_color}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 255, 0), 2)
    cv2.putText(image, f"CX: {cx}, CY: {cy}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0),
                2)

    if selected_color == "yellow":  # Yellow
        color_one, color_two = 1, 1
    elif selected_color == "red":  # Red
        color_one, color_two = 0, 1
    elif selected_color == "blue":  # Blue
        color_one, color_two = 1, 0
    else:
        color_one, color_two = 0, 0  # Default (unknown color)
    # Prepare output
    llpython = [wrapped_angle, cx, cy, color_one, color_two, 0, 0, 0]

    return closest_contour, image, llpython
