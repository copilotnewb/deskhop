# kvm_gaze_switch.py
# Requires: opencv-python, mediapipe, numpy, pyautogui
import cv2, time
import mediapipe as mp
import numpy as np
from collections import deque
import pyautogui
import subprocess, sys

DESKHOP_SCRIPT = "deskhop_switch_ctypes.py"
VID = "0x1209"
PID = "0xC000"

last_label = None

mp_face = mp.solutions.face_mesh

# helper: 3D model points for solvePnP (approximate face model)
MODEL_POINTS = np.array([
    (0.0, 0.0, 0.0),        # nose tip (we will map to landmark)
    (0.0, -63.6, -12.5),    # chin
    (-43.3, 32.7, -26.0),   # left eye corner
    (43.3, 32.7, -26.0),    # right eye corner
    (-28.9, -28.9, -24.1),  # left mouth corner
    (28.9, -28.9, -24.1)    # right mouth corner
], dtype=np.float64)

# indices for MediaPipe landmarks (2D)
# nose tip ~ 1 or 4? We'll use 1 (approx). Confirm empirically if needed.
LM_IDX = {
  'nose_tip': 1,
  'chin': 152,
  'left_eye_outer': 33,
  'right_eye_outer': 263,
  'mouth_left': 61,
  'mouth_right': 291
}
LEFT_IRIS_IDX = 468
RIGHT_IRIS_IDX = 473
LEFT_EYE_INNER = 133
LEFT_EYE_OUTER = 33
RIGHT_EYE_INNER = 362
RIGHT_EYE_OUTER = 263

# thresholds & params (tune in calibration)
YAW_DEG_THRESHOLD = 12.0    # if abs(yaw) > this, trust head
LEFT_EYE_LOW = 0.35
RIGHT_EYE_HIGH = 0.65
DEBOUNCE_FRAMES = 6        # require N consecutive frames of same label
SMOOTH_N = 6

history_x = deque(maxlen=SMOOTH_N)
label_history = deque(maxlen=DEBOUNCE_FRAMES)
last_sent_label = None

def get_2d_points(landmarks, w, h):
    pts = []
    for k in ('nose_tip','chin','left_eye_outer','right_eye_outer','mouth_left','mouth_right'):
        lm = landmarks[LM_IDX[k]]
        pts.append((lm.x * w, lm.y * h))
    return np.array(pts, dtype=np.float64)

def solve_head_pose(landmarks, w, h):
    image_points = get_2d_points(landmarks, w, h)
    focal_length = w
    center = (w/2, h/2)
    camera_matrix = np.array([[focal_length, 0, center[0]],
                              [0, focal_length, center[1]],
                              [0, 0, 1]], dtype=np.float64)
    dist_coeffs = np.zeros((4,1))  # assume no lens distortion
    success, rotation_vector, translation_vector = cv2.solvePnP(MODEL_POINTS, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
    if not success:
        return None
    rmat, _ = cv2.Rodrigues(rotation_vector)
    # yaw, pitch, roll from rotation matrix
    sy = np.sqrt(rmat[0,0]*rmat[0,0] + rmat[1,0]*rmat[1,0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(rmat[2,1], rmat[2,2])
        y = np.arctan2(-rmat[2,0], sy)
        z = np.arctan2(rmat[1,0], rmat[0,0])
    else:
        x = np.arctan2(-rmat[1,2], rmat[1,1])
        y = np.arctan2(-rmat[2,0], sy)
        z = 0
    # convert to degrees: yaw (y), pitch (x), roll (z)
    return np.degrees([y, x, z])  # yaw, pitch, roll

def classify(fused_x, yaw_deg, left_thresh=LEFT_EYE_LOW, right_thresh=RIGHT_EYE_HIGH):
    # prefer head yaw if large
    if abs(yaw_deg) > YAW_DEG_THRESHOLD:
        return "LEFT" if yaw_deg < 0 else "RIGHT"  # sign depends on solvePnP orientation; adjust if needed
    # else use eyes
    if fused_x < left_thresh:
        return "LEFT"
    if fused_x > right_thresh:
        return "RIGHT"
    return "CENTER"

def send_kvm_command(label):
    """Send HID command via deskhop_switch_ctypes.py if it differs from last sent."""
    global last_label

    if label not in ("LEFT", "RIGHT"):
        return  # ignore CENTER or unknown

    # Only send if this is a new screen
    if label == last_label:
        return

    cmd = "b" if label == "LEFT" else "a"

    try:
        subprocess.run(
            [sys.executable, DESKHOP_SCRIPT,
             "--vid", VID, "--pid", PID, "--cmd", cmd],
            check=True,
            capture_output=True
        )
        print(f"Switched DeskHop to {label} ({cmd})")
        last_label = label  # update cached state
    except subprocess.CalledProcessError as e:
        print("DeskHop send failed:", e.stderr.decode(errors="ignore"))

cap = cv2.VideoCapture(0)
with mp_face.FaceMesh(static_image_mode=False, refine_landmarks=True,
                      max_num_faces=1, min_detection_confidence=0.5, min_tracking_confidence=0.5) as fm:
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        h, w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = fm.process(rgb)
        label = "NO_FACE"
        if res.multi_face_landmarks:
            lm = res.multi_face_landmarks[0].landmark
            # head pose
            pose = solve_head_pose(lm, w, h)
            yaw = pose[0] if pose is not None else 0.0

            # iris centres
            left_iris = lm[LEFT_IRIS_IDX]
            right_iris = lm[RIGHT_IRIS_IDX]
            # left eye corners
            le_outer = lm[LEFT_EYE_OUTER]; le_inner = lm[LEFT_EYE_INNER]
            # normalized left iris inside eye bbox
            eye_x0 = le_outer.x * w; eye_x1 = le_inner.x * w
            iris_x = left_iris.x * w
            norm_left = (iris_x - min(eye_x0, eye_x1)) / (abs(eye_x1 - eye_x0) + 1e-6)

            # right eye (mirror coords)
            re_outer = lm[RIGHT_EYE_OUTER]; re_inner = lm[RIGHT_EYE_INNER]
            eye_x0_r = re_inner.x * w; eye_x1_r = re_outer.x * w
            iris_x_r = right_iris.x * w
            norm_right = (iris_x_r - min(eye_x0_r, eye_x1_r)) / (abs(eye_x1_r - eye_x0_r) + 1e-6)

            # fuse eyes (average)
            fused = (norm_left + (1.0 - norm_right)) / 2.0  # right eye mirrored to same scale
            history_x.append(fused)
            smooth_x = np.mean(history_x)

            label = classify(smooth_x, yaw)

            # debouncing
            label_history.append(label)
            if len(label_history) == DEBOUNCE_FRAMES and all(x == label for x in label_history):
                # stable: trigger
                if label in ("LEFT","RIGHT"):
                    send_kvm_command(label)

            # draw debug
            cv2.putText(frame, f"L:{smooth_x:.2f} YAW:{yaw:.1f} => {label}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

        cv2.imshow("Gaze KVM", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

cap.release()
cv2.destroyAllWindows()
