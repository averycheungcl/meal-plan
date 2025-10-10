import cv2

# Use DirectShow backend on Windows
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

ret, frame = cap.read()
if ret:
    cv2.imshow("Webcam", frame)
    cv2.waitKey(0)

cap.release()
cv2.destroyAllWindows()
