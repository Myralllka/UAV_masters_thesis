import time

import cv2
import sys

(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
print(cv2.__version__)
if __name__ == '__main__':

    tracker = cv2.TrackerMedianFlow_create()

    # Read video
    video = cv2.VideoCapture("working.mp4")

    # Exit if video not opened.
    if not video.isOpened():
        print("Could not open video")
        sys.exit()

    # Read first frame.
    ok, frame = video.read()
    if not ok:
        print('Cannot read video file')
        sys.exit()

    # Define an initial bounding box
    # bbox = (150, 10, 50, 50)
    bbox = (157, 53, 57, 47)

    # Uncomment the line below to select a different bounding box
    # bbox = cv2.selectROI(frame, False)
    print(bbox)
    # cv2.imshow("bbox", bbox)
    # Initialize tracker with first frame and bounding box
    ok = tracker.init(frame, bbox)

    fourcc = cv2.VideoWriter_fourcc(*'H264')
    out = cv2.VideoWriter('output.mkv', fourcc, 20.0, (1280, 720))

    while True:
        # Read a new frame
        # time.sleep(0.1)
        ok, frame = video.read()
        if not ok:
            break

        # Start timer
        timer = cv2.getTickCount()

        # Update tracker
        ok, bbox = tracker.update(frame)

        # Calculate Frames per second (FPS)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);

        # Draw bounding box
        if ok:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
        else:
            # Tracking failure
            cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

        # Display result
        # cv2.imshow("Tracking", frame)
        out.write(frame)
        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27: break
    out.release()
