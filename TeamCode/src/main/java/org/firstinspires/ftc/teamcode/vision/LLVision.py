import cv2
import numpy as np


def runPipeline(image, llrobot):
    # Initialize variables
    closestRect = None
    llpython = [0] * 8  # Initialize with zeros
    angle = 0.0

    # Define YCrCb and HSV thresholds
    lowerYCrCb = np.array([0, 151, 0], dtype=np.uint8)
    upperYCrCb = np.array([255, 255, 255], dtype=np.uint8)
    lowerHSV = np.array([16, 57, 0], dtype=np.uint8)
    upperHSV = np.array([142, 255, 255], dtype=np.uint8)

    # Flag to toggle between YCrCb and HSV
    isBlue = bool(llrobot[0]) if llrobot else False

    # Calculate the image center
    image_center = (image.shape[1] / 2, image.shape[0] / 2)

    # Downscale image for faster processing
    scaled_image = cv2.resize(image, (0, 0), fx=0.5, fy=0.5)

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(scaled_image, (5, 5), 0)

    # Convert image to appropriate color space and threshold
    if isBlue:
        color_converted = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        binary_mask = cv2.inRange(color_converted, lowerHSV, upperHSV)
    else:
        color_converted = cv2.cvtColor(blurred, cv2.COLOR_BGR2YCrCb)
        binary_mask = cv2.inRange(color_converted, lowerYCrCb, upperYCrCb)

    # Find contours
    contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Handle case where no contours are found
    if not contours:
        cv2.putText(image, "No objects detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 0, 255), 2)
        return np.array([]), image, llpython

    # Image center
    height, width = image.shape[:2]
    center_x, center_y = width // 2, height // 2

    # Variables to track the best contour
    best_contour = None
    best_distance = float('inf')

    min_area = 10000
    max_area = 50000

    for contour in contours:
        area = cv2.contourArea(contour)
        if min_area <= area <= max_area:
            # Calculate moments and center of contour
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Calculate distance from image center
                distance = np.sqrt((cx - center_x) ** 2 + (cy - center_y) ** 2)

                # Update the best contour based on the shortest distance
                if distance < best_distance:
                    best_contour = contour
                    best_distance = distance

    # Process best_contour only if one is found
    if best_contour is not None:
        # Draw the best contour
        cv2.drawContours(image, [best_contour], 0, (0, 255, 0), 2)

        # Get rotated bounding box
        rect = cv2.minAreaRect(best_contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        # Extract details from rotated rectangle
        (cx, cy), (w, h), angle = rect
        angle = (angle + 180) % 360 - 180  # Normalize angle to (-180, 180)

        # Draw the rotated bounding box
        cv2.drawContours(image, [box], 0, (255, 0, 0), 2)
        cv2.circle(image, (int(cx), int(cy)), 5, (0, 0, 255), -1)
        cv2.putText(image, f"Angle: {angle:.2f}°", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2)

        # Sample the color at the center point
        center_color = image[cy, cx]  # BGR color format

        # Determine color category (using a slightly more robust method)
        b, g, r = center_color
        if r > 150 and g > 150 and b < 100:  # Yellow
            color_one, color_two = 1, 1
        elif r > 150 and g < 100 and b < 100:  # Red
            color_one, color_two = 0, 1
        elif b > 150 and g < 100 and r < 100:  # Blue
            color_one, color_two = 1, 0
        else:
            color_one, color_two = 0, 0  # Default (unknown color)

        # Wrap angle to specific range
        min_angle = 7
        max_angle = 63
        range_size = max_angle - min_angle + 1
        wrapped_angle = ((-range_size / 90) * angle) + max_angle

        # Update llpython with results
        llpython = [wrapped_angle, cx, cy, color_one, color_two, int(w), int(h), int(area)]
    else:
        # No valid contour found
        llpython = [0, 0, 0, 0, 0, 0, 0, 0]

    return best_contour if best_contour is not None else np.array([]), image, llpython
