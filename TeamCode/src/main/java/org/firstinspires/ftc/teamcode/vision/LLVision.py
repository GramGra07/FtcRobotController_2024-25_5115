import cv2
import numpy as np


def runPipeline(image, llrobot):
    # Initialize variables
    closestRect = None
    llpython = [0] * 8  # Initialize with zeros

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

    if contours:
        # Find the largest contour (assuming it's the most relevant)
        largest_contour = max(contours, key=cv2.contourArea)

        # Fit a rotated rectangle to the largest contour
        rect = cv2.minAreaRect(largest_contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box * 2)  # Scale back to original image size

        # Draw the rectangle
        cv2.drawContours(image, [box], 0, (0, 255, 0), 2)

        # Get the center coordinates
        cx, cy = int(rect[0][0] * 2), int(rect[0][1] * 2)  # Scale back to original image size
        cv2.line(image, (cx - 10, cy), (cx + 10, cy), (0, 0, 255), 2)
        cv2.line(image, (cx, cy - 10), (cx, cy + 10), (0, 0, 255), 2)

        # Calculate the angle of the rectangle
        angle = rect[2]

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

        # Update llpython with results
        llpython = [angle, cx, cy, color_one, color_two, 0, 0, 0]

        # Add visualization text
        cv2.putText(image, f"Angle: {angle:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2)
        cv2.putText(image, f"Center: ({cx}, {cy})", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2)
        cv2.putText(image, f"Color: ({color_one}, {color_two})", (10, 90), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255), 2)
    else:
        # No contours found
        cv2.putText(image, "No objects detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 0, 255), 2)

    return np.array([[]]), image, llpython
