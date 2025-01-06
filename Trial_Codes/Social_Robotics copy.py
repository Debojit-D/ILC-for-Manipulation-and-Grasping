import cv2
import numpy as np
import pandas as pd

# Define RGB ranges and thresholds for each color, including area and distance thresholds
color_configs = {
    "Red": {
        "bgr_lower": np.array([30, 24, 154]),
        "bgr_upper": np.array([91, 74, 204]),
        "position_tolerance": 5,
        "width_tolerance": 10,
        "height_tolerance": 20,
        "aspect_ratio_min": 0.5,
        "aspect_ratio_max": 2.0,
        "area_min": 10,
        "area_max": 10000,
        "distance_tolerance": 500
    },
    "Green": {
        "bgr_lower": np.array([70, 120, 0]),
        "bgr_upper": np.array([190, 210, 60]),
        "position_tolerance": 5,
        "width_tolerance": 20,
        "height_tolerance": 20,
        "aspect_ratio_min": 0.5,
        "aspect_ratio_max": 2.0,
        "area_min": 10,
        "area_max": 5000,
        "distance_tolerance": 500
    },
    "Blue": {
        "bgr_lower": np.array([120, 110, 0]),
        "bgr_upper": np.array([255, 200, 70]),
        "position_tolerance": 5,
        "width_tolerance": 25,
        "height_tolerance": 25,
        "aspect_ratio_min": 0.5,
        "aspect_ratio_max": 2.0,
        "area_min": 2,
        "area_max": 6000,
        "distance_tolerance": 500
    },
    "Black": {
        "bgr_lower": np.array([0, 0, 0]),
        "bgr_upper": np.array([50, 50, 50]),
        "position_tolerance": 5,
        "width_tolerance": 30,
        "height_tolerance": 30,
        "aspect_ratio_min": 0.5,
        "aspect_ratio_max": 2.0,
        "area_min": 10,
        "area_max": 10000,
        "distance_tolerance": 500
    }
}

# Stability and tracking parameters
inactive_threshold = 1000

# X thresholds for detection and tracking
initial_x_min_threshold = 500
initial_x_max_threshold = 900
x_min_threshold = 450
x_max_threshold = 1000

# Dictionary to store detected stable objects
tracked_objects = {}
frame_count = 0
all_distances = []  # To store distances across frames

# Function to calculate Euclidean distance
def calculate_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

# Predefined mean distances as constants
mean_color_distances = [
    {"Object 1": "Black", "Object 2": "Blue", "Mean Distance": 76.303561},
    {"Object 1": "Black", "Object 2": "Green", "Mean Distance": 119.750667},
    {"Object 1": "Black", "Object 2": "Red", "Mean Distance": 67.695731},
    {"Object 1": "Blue", "Object 2": "Green", "Mean Distance": 99.754743},
    {"Object 1": "Blue", "Object 2": "Red", "Mean Distance": 76.071609},
    {"Object 1": "Green", "Object 2": "Red", "Mean Distance": 79.214706},
]
def check_distance_flags(all_distances, frame_count, mean_color_distances, threshold_percent=10): 
    """
    Compare current distances with predefined mean distances and raise flags for thresholds.
    """
    # Convert all_distances to a DataFrame
    df = pd.DataFrame(all_distances)
    
    # Debug: Print the DataFrame structure
    # print("DataFrame structure:")
    # print(df.head())
    
    if "Frame" not in df.columns:
        # print("Error: 'Frame' column not found in DataFrame.")
        return 0, []

    # Filter for the current frame
    current_frame_distances = df[df["Frame"] == frame_count]

    flags_raised = 0
    flagged_pairs = []

    for mean_entry in mean_color_distances:
        obj1 = mean_entry["Object 1"]
        obj2 = mean_entry["Object 2"]
        mean_distance = mean_entry["Mean Distance"]
        threshold = mean_distance * (threshold_percent / 100.0)

        # Check if the color pair exists in the current frame's distances
        match = current_frame_distances[
            ((current_frame_distances["Object 1"] == obj1) & (current_frame_distances["Object 2"] == obj2)) |
            ((current_frame_distances["Object 1"] == obj2) & (current_frame_distances["Object 2"] == obj1))
        ]

        if not match.empty:
            current_distance = match.iloc[0]["Distance"]
            if abs(current_distance - mean_distance) <= threshold:
                flags_raised += 1
                flagged_pairs.append((obj1, obj2))

    return flags_raised, flagged_pairs


def main(use_camera=False, video_path='Week-2/Participant_1/camera1.avi'):
    # Start video capture from camera or file based on `use_camera`
    cap = cv2.VideoCapture(0) if use_camera else cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print("Error: Could not open video source.")
        return

    global frame_count
    # Variables to track frame scores
    frame_scores = []
    # Initialize the message counter
    message_counter = 0
    message = ""  # Message to be displayed

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        frame_count += 1

        # Display frame count on the video
        cv2.putText(frame, f"Frame Count: {frame_count}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Determine x thresholds based on frame count
        if frame_count <= 10:
            x_threshold_min = initial_x_min_threshold
            x_threshold_max = initial_x_max_threshold
        else:
            x_threshold_min = x_min_threshold
            x_threshold_max = x_max_threshold

        # Dictionary to store largest detected object per color in the current frame
        largest_objects = {}

        # Process each color range
        for color, config in color_configs.items():
            mask = cv2.inRange(frame, config["bgr_lower"], config["bgr_upper"])
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Special handling for black objects: Only detect the largest one
            if color == "Black":
                max_black_area = 0
                largest_black_contour = None
                for contour in contours:
                    area = cv2.contourArea(contour)
                    x, y, w, h = cv2.boundingRect(contour)

                    # Apply x_threshold and area filtering
                    if (x_threshold_min <= x <= x_threshold_max and area > max_black_area and area > config["area_min"]):
                        aspect_ratio = w / h if h != 0 else 0
                        if config["aspect_ratio_min"] <= aspect_ratio <= config["aspect_ratio_max"]:
                            max_black_area = area
                            largest_black_contour = contour

                # Store the largest black object
                if largest_black_contour is not None:
                    x, y, w, h = cv2.boundingRect(largest_black_contour)
                    largest_objects[color] = {"position": (x, y, w, h), "area": max_black_area}

            # For other colors, store the largest detected object
            else:
                for contour in contours:
                    if cv2.contourArea(contour) > 500:
                        x, y, w, h = cv2.boundingRect(contour)
                        area = w * h
                        aspect_ratio = w / h if h != 0 else 0

                        # Apply x_threshold for all other colors and keep the largest object per color
                        if (x_threshold_min <= x <= x_threshold_max and
                            config["area_min"] <= area <= config["area_max"] and
                            config["aspect_ratio_min"] <= aspect_ratio <= config["aspect_ratio_max"]):
                            
                            # Store the largest object for this color if it's the largest so far
                            if color not in largest_objects or area > largest_objects[color]["area"]:
                                largest_objects[color] = {"position": (x, y, w, h), "area": area}

        # Column position for displaying centroids on the right
        text_position_y = 40
        text_position_x = frame.shape[1] - 450  # Place text on the right side of the frame

        # Match each detected object with tracked objects
        for color, data in largest_objects.items():
            x, y, w, h = data["position"]
            matched_id = None
            min_distance = config["distance_tolerance"]

            # Find the closest tracked object of the same color
            for obj_id, tracked_obj in tracked_objects.items():
                if tracked_obj["color"] == color:
                    distance = calculate_distance(
                        (x + w / 2, y + h / 2),
                        (tracked_obj["position"][0] + tracked_obj["position"][2] / 2,
                         tracked_obj["position"][1] + tracked_obj["position"][3] / 2)
                    )
                    if distance < min_distance:
                        min_distance = distance
                        matched_id = obj_id

            # Update position if a match is found, otherwise add a new tracked object
            if matched_id is not None:
                tracked_objects[matched_id]["position"] = (x, y, w, h)
                tracked_objects[matched_id]["inactive"] = 0  # Reset inactivity count
            else:
                tracked_objects[frame_count] = {  # Use frame_count as a unique identifier
                    "color": color,
                    "position": (x, y, w, h),
                    "inactive": 0
                }

        # Increment inactive count for objects that were not matched this frame
        for obj_id, tracked_obj in list(tracked_objects.items()):
            if tracked_obj["inactive"] < inactive_threshold:
                tracked_obj["inactive"] += 1
            else:
                # Remove objects that have been inactive for too long
                del tracked_objects[obj_id]

        # Draw bounding boxes and centroids for tracked objects
        for obj_id, obj in tracked_objects.items():
            x, y, w, h = obj["position"]
            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # Draw centroid
            centroid_x = x + w // 2
            centroid_y = y + h // 2
            centroid_color = (0, 0, 255)  # Default color is red

            # Check if this centroid is within the threshold for any pair
            for pair in mean_color_distances:
                obj1, obj2 = pair["Object 1"], pair["Object 2"]
                mean_distance = pair["Mean Distance"]
                threshold_10 = mean_distance * 0.5

                # Match this object with tracked objects for distance comparison
                for obj_id2, obj2_tracked in tracked_objects.items():
                    if obj["color"] in (obj1, obj2) and obj2_tracked["color"] in (obj1, obj2):
                        centroid2_x = obj2_tracked["position"][0] + obj2_tracked["position"][2] // 2
                        centroid2_y = obj2_tracked["position"][1] + obj2_tracked["position"][3] // 2
                        distance = calculate_distance((centroid_x, centroid_y), (centroid2_x, centroid2_y))

                        # If within threshold, set color to yellow
                        if abs(distance - mean_distance) <= threshold_10:
                            centroid_color = (0, 255, 255)  # Yellow for within threshold
            cv2.circle(frame, (centroid_x, centroid_y), 5, centroid_color, -1)  # Red dot for centroid

            # Display the centroid position in red on the right side
            cv2.putText(frame, f"{obj['color']} centroid: ({centroid_x}, {centroid_y})",
                        (text_position_x, text_position_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, ( 255, 0, 0), 1)
            text_position_y += 20  # Move to the next line for each object
            # Draw text label
            cv2.putText(frame, f"{obj['color']} Object", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
        # Calculate distances between objects and add to all_distances
        calculated_pairs = set()  # To track which pairs have already been calculated

        for obj_id1, obj1 in tracked_objects.items():
            for obj_id2, obj2 in tracked_objects.items():
                if obj_id1 != obj_id2:
                    # Use the color values instead of object IDs for display
                    color1 = obj1["color"]
                    color2 = obj2["color"]

                    # Sort the pair to ensure uniqueness (e.g., ('Red', 'Black') == ('Black', 'Red'))
                    pair = tuple(sorted([color1, color2]))
                    if pair not in calculated_pairs:
                        calculated_pairs.add(pair)  # Mark this pair as calculated

                        # Calculate the centroid and distance
                        centroid1 = (obj1["position"][0] + obj1["position"][2] // 2,
                                    obj1["position"][1] + obj1["position"][3] // 2)
                        centroid2 = (obj2["position"][0] + obj2["position"][2] // 2,
                                    obj2["position"][1] + obj2["position"][3] // 2)
                        distance = calculate_distance(centroid1, centroid2)

                        # Append to all_distances
                        all_distances.append({
                            "Frame": frame_count,
                            "Object 1": color1,
                            "Object 2": color2,
                            "Distance": distance
                        })

                        text_position_y += 20  # Move to the next line for each object
                        cv2.putText(frame, f"{color1}-{color2}: {distance:.2f}",
                            (text_position_x, text_position_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                        
        # Check distances for 10% threshold
        flags_10, flagged_pairs_10 = check_distance_flags(all_distances, frame_count, mean_color_distances, threshold_percent=50)
        score_10 = 15 * flags_10

        # Check distances for 5% threshold
        flags_5, flagged_pairs_5 = check_distance_flags(all_distances, frame_count, mean_color_distances, threshold_percent=45)
        score_5 = 100 if flags_5 == len(mean_color_distances) else 0

        # Define score variable
        score = score_10

        # Display score_10
        text_position_y += 20
        cv2.putText(frame, f"Score: {score}/100", (text_position_x, text_position_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Display score_5 only if it is 100
        text_position_y += 20
        if score_5 == 100:
            # Trigger the perfect score message for 100 frames
            if message != "Perfect Score achieved for 100 frames. You win a pizza from Madhu Sir":
                message = "Perfect Score achieved for 100 frames. You win a pizza from Madhu Sir"
                message_counter = 100  # Display for 100 frames

        # Print results
        # print(f"Frame {frame_count}:")
        # print(f"10% Threshold - Flags: {flags_10}, Score: {score_10}, Flagged Pairs: {flagged_pairs_10}")
        # print(f"5% Threshold - Flags: {flags_5}, Score: {score_5}, Flagged Pairs: {flagged_pairs_5}")

        # Display the message if the counter is active
        if message_counter > 0:
            cv2.putText(frame, message, (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            message_counter -= 1  # Decrement the counter

        # Append the score for the current frame
        frame_scores.append(score_10)

        # Trigger the message at specific frame intervals
        if frame_count == 1000:
            message = "Come on. You can do it. I will tell your progress."
            message_counter = 200  # Display for 200 frames

        elif frame_count % 2000 == 0 and frame_count >= 4000:
            # Calculate mean scores for intervals
            current_interval_start = frame_count - 2000
            previous_interval_start = max(0, frame_count - 4000)

            current_interval_mean = np.mean(frame_scores[current_interval_start+1200:frame_count])
            previous_interval_mean = np.mean(frame_scores[previous_interval_start+1200:current_interval_start])

            # Determine the message
            if current_interval_mean > previous_interval_mean:
                message = "It seems you are going good."
            else:
                message = "Are you taking them apart for a reason?"
            message_counter = 200  # Display for 200 frames

        # Show the video with tracking
        cv2.imshow('Video', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # Exit on 'q' key press
            break
        elif key == ord('p'):  # Pause on 'p' key press
            print("Paused. Press 'p' to resume.")
            while True:
                # Wait indefinitely until 'p' is pressed again to resume
                key = cv2.waitKey(1) & 0xFF
                if key == ord('p'):
                    print("Resumed.")
                    break  # Exit the pause loop and resume the main loop
                elif key == ord('q'):  # Allow exiting while paused
                    cap.release()
                    cv2.destroyAllWindows()
                    exit()

    # # Save all distances to a single Excel file
    # if all_distances:
    #     df = pd.DataFrame(all_distances)
    #     df.to_excel(output_file, index=False)
    #     print(f"Saved all distances to {output_file}")

    # # After appending the distance to all_distances
    # mean_distances = calculate_mean_distances(all_distances, frame_count, frame_window=150)

    # # Print mean distances to the console (optional)
    # print(mean_distances)

 
    # Release and close
    cap.release()
    cv2.destroyAllWindows()

# Run the main function
main(use_camera=False)  # Set to True to use the camera feed instead of video


