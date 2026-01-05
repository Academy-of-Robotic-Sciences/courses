---
id: hum-109-ai-behaviors
title: "HUM-109: AI-Powered Behaviors"
sidebar_position: 10
version: 2.0.0
---
<link rel="stylesheet" href="/styles/textbook.css" />

# Chapter 9: AI-Powered Adaptive Behaviors

<div class="chapter-header">
<div class="course-info">

| **Course Code** | HUM-109 |
|---|---|
| **Duration** | 8 hours (full day) |
| **Level** | Advanced AI Integration |
| **Prerequisites** | HUM-108 (Manipulation & Coordination) |
| **Primary Role** | AI Engineer |
| **Secondary Role** | Software Engineer, Systems Engineer |

</div>

<div class="abstract">
**Abstract**: This chapter transitions humanoid robots from pre-programmed behaviors to adaptive, intelligent systems through machine learning integration. We explore computer vision for environmental perception, imitation learning for behavior acquisition, reinforcement learning for optimization, and natural language processing for human-robot interaction. Students implement complete AI pipelines including YOLO-based object detection, LeRobot imitation learning, gait optimization through reinforcement learning, and voice-controlled task execution, culminating in autonomous behavior generation through behavior trees.
</div>
</div>

## 9.1 Introduction to AI in Humanoid Robotics

The integration of artificial intelligence transforms robots from deterministic systems executing pre-programmed sequences into adaptive agents capable of learning, generalization, and autonomous decision-making. This paradigm shift addresses fundamental limitations of traditional control approaches: brittleness to environmental variation, inability to improve from experience, and limited generalization to novel situations.

### 9.1.1 Historical Evolution

The application of AI to robotics has evolved through several distinct phases:

**Classical AI Era (1960s-1980s)**: Early robotics relied on symbolic AI and expert systems. Shakey the Robot (1966) at SRI demonstrated basic planning and navigation using logical reasoning. However, the "symbol grounding problem" and computational limitations restricted practical deployment.

**Behavior-Based Robotics (1980s-1990s)**: Rodney Brooks' subsumption architecture (1986) introduced reactive, behavior-based control that operated without explicit world models. This approach proved more robust for real-world deployment but lacked learning capabilities.

**Learning-Based Approaches (1990s-2010s)**: Statistical machine learning methods, particularly reinforcement learning (Sutton & Barto, 1998) and supervised learning for vision (Viola-Jones face detection, 2001), enabled robots to improve from experience. However, hand-engineered features limited performance.

**Deep Learning Revolution (2012-present)**: Convolutional neural networks for vision (AlexNet, 2012), recurrent networks for sequences, and deep reinforcement learning (DQN, 2015; AlphaGo, 2016) dramatically improved perception and control. Modern humanoid AI leverages these advances through frameworks like LeRobot, which combines vision transformers with behavior cloning.

### 9.1.2 The AI Pipeline for Humanoid Robots

A complete AI system for humanoid robotics comprises four integrated components:

<div class="equation-block">

**Perception → Understanding → Planning → Execution**

</div>

**Perception**: Computer vision (object detection, pose estimation, scene segmentation) and sensor fusion transform raw sensory data into structured representations.

**Understanding**: Semantic reasoning interprets perceptual data in task context, recognizing objects, inferring intent, and predicting outcomes.

**Planning**: High-level decision-making selects actions to achieve goals, considering constraints, resources, and uncertainty.

**Execution**: Low-level control translates plans into motor commands, integrating with the hierarchical control systems developed in previous chapters.

### 9.1.3 Machine Learning Paradigms

Three primary learning paradigms apply to humanoid robotics:

**Supervised Learning**: Learn mapping f: X → Y from labeled examples {(x₁, y₁), ..., (xₙ, yₙ)}. Applications include object classification, pose estimation, and grasp prediction.

**Reinforcement Learning**: Learn policy π(a|s) maximizing cumulative reward:

<div class="equation-block">

$$
\max_{\pi} \mathbb{E}_{\tau \sim \pi} \left[ \sum_{t=0}^{T} \gamma^t r(s_t, a_t) \right]
$$

</div>

where s_t is state, a_t is action, r is reward, and γ is discount factor. Applications include gait optimization, manipulation skill learning, and task planning.

**Imitation Learning**: Learn policy from expert demonstrations without explicit reward function. Behavioral cloning learns π(a|s) by supervised learning from expert state-action pairs:

<div class="equation-block">

$$
\min_{\pi} \mathbb{E}_{(s,a) \sim \mathcal{D}_{\text{expert}}} \left[ \mathcal{L}(\pi(s), a) \right]
$$

</div>

where 𝓓_expert is the demonstration dataset and 𝓛 is a loss function (typically MSE for continuous actions).

<div class="quiz">

**Quiz 9.1: AI Fundamentals**

1. What is the primary advantage of imitation learning over reinforcement learning for humanoid robots?
   - a) Faster training time
   - b) No need to design reward functions; learn directly from demonstrations
   - c) Better generalization
   - d) Lower computational cost

2. In the expression E_τ~π[Σγᵗr(sₜ,aₜ)], what does the discount factor γ < 1 represent?
   - a) Computational efficiency
   - b) Preference for immediate rewards over distant future rewards
   - c) Measurement noise
   - d) Learning rate

3. Why did deep learning revolutionize robot perception compared to hand-engineered features?
   - a) Faster inference
   - b) Automatic feature learning from data captures relevant patterns better
   - c) Lower memory requirements
   - d) Simpler implementation

**Answers**: 1-b, 2-b, 3-b

</div>

---

## 9.2 Computer Vision for Humanoid Perception

Computer vision enables robots to perceive and understand their environment through visual sensing. Modern approaches leverage deep convolutional neural networks that learn hierarchical feature representations directly from pixel data.

### 9.2.1 Object Detection Theory

Object detection localizes and classifies objects in images. The YOLO (You Only Look Once) family of detectors frames detection as regression:

**Problem Formulation**: Given input image I ∈ ℝ^(H×W×3), predict:
- Bounding boxes B_i = (x, y, w, h) for i = 1...N objects
- Class probabilities P_i ∈ ℝ^C for C classes
- Confidence scores c_i ∈ [0,1]

**YOLO Architecture**: The network divides the image into S×S grid. Each grid cell predicts B bounding boxes and C class probabilities. The prediction tensor is:

<div class="equation-block">

$$
\mathbf{Y} \in \mathbb{R}^{S \times S \times (B \cdot 5 + C)}
$$

</div>

where each bounding box has 5 parameters: (x, y, w, h, confidence).

**Loss Function**: YOLO optimizes multi-part loss combining localization, confidence, and classification:

<div class="equation-block">

$$
\begin{aligned}
\mathcal{L} = &\lambda_{\text{coord}} \sum_{i=0}^{S^2} \sum_{j=0}^{B} \mathbb{1}_{ij}^{\text{obj}} [(x_i - \hat{x}_i)^2 + (y_i - \hat{y}_i)^2] \\
&+ \lambda_{\text{coord}} \sum_{i=0}^{S^2} \sum_{j=0}^{B} \mathbb{1}_{ij}^{\text{obj}} [(\sqrt{w_i} - \sqrt{\hat{w}_i})^2 + (\sqrt{h_i} - \sqrt{\hat{h}_i})^2] \\
&+ \sum_{i=0}^{S^2} \sum_{j=0}^{B} \mathbb{1}_{ij}^{\text{obj}} (C_i - \hat{C}_i)^2 \\
&+ \lambda_{\text{noobj}} \sum_{i=0}^{S^2} \sum_{j=0}^{B} \mathbb{1}_{ij}^{\text{noobj}} (C_i - \hat{C}_i)^2 \\
&+ \sum_{i=0}^{S^2} \mathbb{1}_{i}^{\text{obj}} \sum_{c \in \text{classes}} (p_i(c) - \hat{p}_i(c))^2
\end{aligned}
$$

</div>

where 𝟙^obj_ij indicates if object appears in cell i, box j, and λ weights balance different loss terms.

### 9.2.2 3D Pose Estimation from 2D Detection

For manipulation, we need 3D object positions. The Perspective-n-Point (PnP) problem estimates object pose from 2D-3D correspondences.

**Mathematical Framework**: Given n correspondences {(**p**_i, **P**_i)} where **p**_i = (u_i, v_i) are 2D image points and **P**_i = (X_i, Y_i, Z_i) are 3D object points, find rotation **R** and translation **t** such that:

<div class="equation-block">

$$
\lambda_i \begin{bmatrix} u_i \\ v_i \\ 1 \end{bmatrix} = \mathbf{K} [\mathbf{R} | \mathbf{t}] \begin{bmatrix} X_i \\ Y_i \\ Z_i \\ 1 \end{bmatrix}
$$

</div>

For known object dimensions, we can estimate depth from apparent size as shown in Chapter 8.

---

## 9.3 Guided Project: Vision-Based Object Recognition

### Objective
Implement a real-time object detection and 3D localization system using YOLO, enabling the robot to perceive and localize objects for autonomous manipulation.

### 9.3.1 Implementation: YOLO-Based Detection

```python
import cv2
import numpy as np
from ultralytics import YOLO
import torch

class RobotVisionSystem:
    """
    Advanced computer vision system for humanoid robot perception.

    Implements:
    - Real-time object detection using YOLOv8
    - 3D position estimation from monocular vision
    - Multi-object tracking across frames
    - Semantic scene understanding
    """

    def __init__(self, camera_id=0, model_size='n', device='cuda'):
        """
        Initialize vision system.

        Args:
            camera_id: Camera device index
            model_size: YOLO model size ('n', 's', 'm', 'l', 'x')
            device: Compute device ('cuda' or 'cpu')
        """
        # Initialize camera
        self.camera = cv2.VideoCapture(camera_id)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.camera.set(cv2.CAP_PROP_FPS, 30)

        # Load YOLO model
        self.device = device if torch.cuda.is_available() else 'cpu'
        self.model = YOLO(f'yolov8{model_size}.pt')
        self.model.to(self.device)

        # Camera calibration matrix (from calibration)
        self.K = np.array([
            [615.0, 0.0, 320.0],
            [0.0, 615.0, 240.0],
            [0.0, 0.0, 1.0]
        ])

        # Object database with physical dimensions (in cm)
        self.object_db = {
            'cup': {'height': 10.0, 'width': 8.0, 'depth': 8.0},
            'bottle': {'height': 20.0, 'width': 6.0, 'depth': 6.0},
            'book': {'height': 20.0, 'width': 15.0, 'depth': 2.0},
            'cell phone': {'height': 14.0, 'width': 7.0, 'depth': 1.0},
            'apple': {'height': 8.0, 'width': 8.0, 'depth': 8.0},
            'mouse': {'height': 3.0, 'width': 6.0, 'depth': 10.0}
        }

        # Tracking state
        self.tracked_objects = {}
        self.next_object_id = 0

    def detect_objects(self, confidence_threshold=0.5, iou_threshold=0.4):
        """
        Detect objects in current camera frame.

        Args:
            confidence_threshold: Minimum detection confidence
            iou_threshold: IoU threshold for NMS

        Returns:
            detections: List of detected objects with metadata
        """
        # Capture frame
        ret, frame = self.camera.read()
        if not ret:
            return []

        # Run inference
        results = self.model.predict(
            frame,
            conf=confidence_threshold,
            iou=iou_threshold,
            verbose=False
        )

        detections = []
        for r in results:
            boxes = r.boxes

            for box in boxes:
                # Extract detection data
                xyxy = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]

                # Compute center and dimensions
                x1, y1, x2, y2 = xyxy
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                width = x2 - x1
                height = y2 - y1

                detection = {
                    'class': cls_name,
                    'confidence': conf,
                    'bbox': xyxy,
                    'center_2d': np.array([center_x, center_y]),
                    'size_2d': np.array([width, height]),
                    'frame': frame.copy()
                }

                # Estimate 3D position if object is in database
                if cls_name in self.object_db:
                    pos_3d = self.estimate_3d_position(
                        xyxy, cls_name
                    )
                    detection['position_3d'] = pos_3d
                    detection['distance'] = pos_3d[2]

                detections.append(detection)

        return detections

    def estimate_3d_position(self, bbox, object_class):
        """
        Estimate 3D position using known object size.

        Method: Use pinhole camera model with known object dimensions
        to estimate depth, then backproject to 3D.

        Args:
            bbox: Bounding box [x1, y1, x2, y2]
            object_class: Object type from database

        Returns:
            position_3d: [X, Y, Z] in camera frame (cm)
        """
        if object_class not in self.object_db:
            # Unknown object - use default size
            obj_height = 10.0
        else:
            obj_height = self.object_db[object_class]['height']

        # Image measurements
        x1, y1, x2, y2 = bbox
        h_img = y2 - y1  # Height in pixels

        # Depth from object size (pinhole model)
        # Z = (f * H_real) / h_img
        f_y = self.K[1, 1]
        Z = (f_y * obj_height) / h_img

        # Backproject center to 3D
        u_c = (x1 + x2) / 2
        v_c = (y1 + y2) / 2

        c_x, c_y = self.K[0, 2], self.K[1, 2]
        f_x, f_y = self.K[0, 0], self.K[1, 1]

        X = (u_c - c_x) * Z / f_x
        Y = (v_c - c_y) * Z / f_y

        return np.array([X, Y, Z])

    def track_objects(self, detections, max_distance=50):
        """
        Track objects across frames using simple distance-based association.

        Args:
            detections: Current frame detections
            max_distance: Maximum distance for association (pixels)

        Returns:
            tracked: Detections with tracking IDs
        """
        tracked = []

        # Match detections to existing tracks
        unmatched_detections = list(range(len(detections)))
        matched_tracks = set()

        for det_idx in range(len(detections)):
            det = detections[det_idx]

            # Find closest existing track
            min_dist = max_distance
            best_track_id = None

            for track_id, track in self.tracked_objects.items():
                if track['class'] != det['class']:
                    continue

                # Distance in image space
                dist = np.linalg.norm(
                    det['center_2d'] - track['center_2d']
                )

                if dist < min_dist:
                    min_dist = dist
                    best_track_id = track_id

            if best_track_id is not None:
                # Update existing track
                det['track_id'] = best_track_id
                self.tracked_objects[best_track_id] = det
                matched_tracks.add(best_track_id)
                unmatched_detections.remove(det_idx)

            tracked.append(det)

        # Create new tracks for unmatched detections
        for det_idx in unmatched_detections:
            det = detections[det_idx]
            track_id = self.next_object_id
            self.next_object_id += 1

            det['track_id'] = track_id
            self.tracked_objects[track_id] = det

        # Remove old tracks
        active_track_ids = matched_tracks | {
            det['track_id'] for det in tracked
            if 'track_id' in det
        }

        old_tracks = set(self.tracked_objects.keys()) - active_track_ids
        for track_id in old_tracks:
            del self.tracked_objects[track_id]

        return tracked

    def find_nearest_object(self, target_class=None):
        """
        Find nearest object, optionally of specific class.

        Args:
            target_class: Specific object class, or None for any

        Returns:
            nearest: Nearest object detection, or None
        """
        detections = self.detect_objects()

        if target_class:
            detections = [d for d in detections
                         if d['class'] == target_class]

        if not detections:
            return None

        # Find closest by 3D distance
        detections_with_3d = [d for d in detections
                             if 'position_3d' in d]

        if not detections_with_3d:
            return None

        nearest = min(detections_with_3d,
                     key=lambda d: d['distance'])

        return nearest
```

**Theoretical Analysis**:

The depth estimation accuracy depends on several factors:

1. **Object size accuracy**: Error in known dimensions directly scales depth error
2. **Bounding box accuracy**: Detection noise affects pixel measurements
3. **Focal length calibration**: Camera calibration errors propagate

**Error Analysis**: For depth Z, the estimated error δZ is:

<div class="equation-block">

$$
\frac{\delta Z}{Z} = \sqrt{\left(\frac{\delta f}{f}\right)^2 + \left(\frac{\delta H}{H}\right)^2 + \left(\frac{\delta h}{h}\right)^2}
$$

</div>

where δf, δH, δh are errors in focal length, object height, and image height respectively.

For typical values (f = 600px, H = 10cm, h = 100px) with 5% errors: δZ/Z ≈ 8.7%, giving ±4.4cm error at 50cm distance.

### 9.3.2 Testing and Validation

```python
def test_vision_system():
    """
    Comprehensive test of vision system capabilities.
    """
    vision = RobotVisionSystem()

    print("Vision System Test Suite")
    print("=" * 50)

    # Test 1: Detection accuracy
    print("\n1. Detection Accuracy Test")
    for i in range(10):
        detections = vision.detect_objects()
        print(f"Frame {i}: Detected {len(detections)} objects")

        for det in detections:
            print(f"  - {det['class']}: conf={det['confidence']:.2f}, "
                  f"dist={det.get('distance', 'N/A')}cm")

        time.sleep(0.5)

    # Test 2: 3D position estimation
    print("\n2. 3D Position Estimation Test")
    target = vision.find_nearest_object('cup')

    if target:
        print(f"Cup detected at: {target['position_3d']} cm")
        print(f"Confidence: {target['confidence']:.2f}")
    else:
        print("No cup detected")

    # Test 3: Tracking consistency
    print("\n3. Tracking Consistency Test")
    for i in range(20):
        detections = vision.detect_objects()
        tracked = vision.track_objects(detections)

        print(f"Frame {i}: {len(tracked)} tracked objects")
        for obj in tracked:
            if 'track_id' in obj:
                print(f"  ID {obj['track_id']}: {obj['class']}")

        time.sleep(0.1)
```

<div class="quiz">

**Quiz 9.2: Computer Vision**

1. In the YOLO loss function, why do we use √w and √h rather than w and h directly for size regression?
   - a) Computational efficiency
   - b) To weight small and large boxes more equally
   - c) To ensure positive values
   - d) Historical convention

2. If an object's true height is 10cm but we assume 12cm, how does this affect estimated depth?
   - a) Depth is overestimated by 20%
   - b) Depth is underestimated by 20%
   - c) No effect on depth
   - d) Depth error is nonlinear

3. What is the primary advantage of tracking objects across frames?
   - a) Faster detection
   - b) Temporal consistency and ability to predict motion
   - c) Better classification accuracy
   - d) Lower memory usage

**Answers**: 1-b, 2-a (Z = fH/h, so if H→1.2H, then Z→1.2Z), 3-b

</div>

---

## 9.4 Imitation Learning with LeRobot

Imitation learning enables robots to acquire skills from human demonstrations without explicit programming. The LeRobot framework provides tools for data collection, policy learning, and deployment.

### 9.4.1 Behavioral Cloning Theory

Behavioral cloning treats imitation as supervised learning. Given expert demonstrations 𝓓 = {(s₁, a₁), (s₂, a₂), ..., (sₙ, aₙ)}, learn policy π_θ(a|s) by minimizing:

<div class="equation-block">

$$
\mathcal{L}(\theta) = \mathbb{E}_{(s,a) \sim \mathcal{D}} \left[ \|a - \pi_\theta(s)\|^2 \right]
$$

</div>

**Distributional Shift Problem**: A key challenge is that the learned policy may visit states not in the demonstration data, leading to compounding errors. If the policy makes a small error early, it reaches states increasingly far from the demonstrated distribution.

**Mathematical Analysis**: Let p_π(s_t) be the state distribution under policy π at time t. The error grows as:

<div class="equation-block">

$$
\mathbb{E}_{s_t \sim p_{\pi}}\left[\ell(s_t, a_t)\right] \leq \epsilon_{\text{BC}} + t \cdot \epsilon_{\text{shift}}
$$

</div>

where ε_BC is the behavioral cloning error and ε_shift quantifies distributional shift per timestep.

**Mitigation Strategies**:
1. **DAgger** (Dataset Aggregation): Iteratively collect data on-policy and retrain
2. **Robust features**: Use state representations invariant to small perturbations
3. **Conservative policies**: Add noise during training to improve coverage

### 9.4.2 Transformer-Based Policy Networks

Modern imitation learning uses transformer architectures for temporal modeling:

<div class="equation-block">

**Input**: Observation sequence {o₁, o₂, ..., o_T}
**Encoder**: Multi-head self-attention processes temporal dependencies
**Output**: Action sequence {a₁, a₂, ..., a_T}

</div>

The self-attention mechanism computes:

<div class="equation-block">

$$
\text{Attention}(Q, K, V) = \text{softmax}\left(\frac{QK^T}{\sqrt{d_k}}\right)V
$$

</div>

where Q (query), K (key), V (value) are linear projections of the input observations.

---

## 9.5 Guided Project: Imitation Learning Pipeline

### Objective
Implement a complete imitation learning system that records expert demonstrations, trains a policy network, and deploys learned behaviors on the humanoid robot.

### 9.5.1 Implementation: Data Collection

```python
import numpy as np
import torch
import torch.nn as nn
from collections import deque
import pickle
import time

class DemonstrationRecorder:
    """
    Record expert demonstrations for imitation learning.

    Captures:
    - Visual observations (camera images)
    - Proprioceptive state (joint positions, velocities)
    - Actions (joint commands)
    - Timestamps for temporal alignment
    """

    def __init__(self, robot, camera, save_dir='demonstrations/'):
        self.robot = robot
        self.camera = camera
        self.save_dir = save_dir

        # Create save directory
        import os
        os.makedirs(save_dir, exist_ok=True)

        self.recording = False
        self.current_demo = None

    def start_recording(self, task_name):
        """
        Begin recording a demonstration.

        Args:
            task_name: Name/description of demonstrated task
        """
        self.recording = True
        self.current_demo = {
            'task': task_name,
            'observations': [],
            'actions': [],
            'timestamps': [],
            'metadata': {
                'start_time': time.time(),
                'robot_config': self.robot.get_config()
            }
        }

        print(f"Recording started: {task_name}")
        print("Press 'q' to stop recording")

    def record_step(self):
        """
        Record single timestep during demonstration.
        """
        if not self.recording:
            return

        # Capture observations
        observation = {
            # Visual
            'image': self.camera.read(),

            # Proprioceptive
            'joint_positions': self.robot.get_joint_positions(),
            'joint_velocities': self.robot.get_joint_velocities(),
            'joint_torques': self.robot.get_joint_torques(),

            # IMU
            'imu_orientation': self.robot.get_imu_orientation(),
            'imu_angular_velocity': self.robot.get_imu_angular_velocity(),

            # Force sensors
            'foot_forces': self.robot.get_foot_forces()
        }

        # Get action (change in joint positions)
        if len(self.current_demo['observations']) > 0:
            prev_pos = self.current_demo['observations'][-1]['joint_positions']
            curr_pos = observation['joint_positions']
            action = curr_pos - prev_pos
        else:
            action = np.zeros_like(observation['joint_positions'])

        # Store data
        self.current_demo['observations'].append(observation)
        self.current_demo['actions'].append(action)
        self.current_demo['timestamps'].append(time.time())

    def stop_recording(self):
        """
        Stop recording and save demonstration.
        """
        if not self.recording:
            return

        self.recording = False

        # Add metadata
        self.current_demo['metadata']['end_time'] = time.time()
        self.current_demo['metadata']['duration'] = (
            self.current_demo['metadata']['end_time'] -
            self.current_demo['metadata']['start_time']
        )
        self.current_demo['metadata']['num_steps'] = len(
            self.current_demo['observations']
        )

        # Save to file
        filename = f"{self.save_dir}/demo_{int(time.time())}.pkl"
        with open(filename, 'wb') as f:
            pickle.dump(self.current_demo, f)

        print(f"\nRecording saved: {filename}")
        print(f"Duration: {self.current_demo['metadata']['duration']:.1f}s")
        print(f"Steps: {self.current_demo['metadata']['num_steps']}")

        return filename

    def record_interactive(self, task_name):
        """
        Record demonstration with interactive control.

        Human controls robot (e.g., through teleoperation or
        physical guidance), and system records state-action pairs.
        """
        import keyboard

        self.start_recording(task_name)

        print("\nControl the robot to demonstrate the task")
        print("Recording at 50 Hz")

        try:
            while self.recording:
                # Record current state
                self.record_step()

                # Check for stop signal
                if keyboard.is_pressed('q'):
                    break

                # 50 Hz recording rate
                time.sleep(0.02)

        except KeyboardInterrupt:
            pass

        return self.stop_recording()


class BehaviorCloningPolicy(nn.Module):
    """
    Neural network policy for behavior cloning.

    Architecture:
    - Visual encoder (CNN) processes camera images
    - Proprioceptive encoder (MLP) processes joint states
    - Fusion layer combines modalities
    - Temporal encoder (LSTM/Transformer) models sequences
    - Action decoder outputs joint commands
    """

    def __init__(self, image_shape=(3, 224, 224), proprio_dim=22,
                 action_dim=22, hidden_dim=256):
        super().__init__()

        # Visual encoder (simple CNN)
        self.visual_encoder = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            nn.ReLU(),
            nn.Flatten(),
            nn.Linear(64 * 24 * 24, 512),  # Adjust based on image size
            nn.ReLU()
        )

        # Proprioceptive encoder
        self.proprio_encoder = nn.Sequential(
            nn.Linear(proprio_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU()
        )

        # Fusion and temporal processing
        self.fusion = nn.Linear(512 + 128, hidden_dim)

        # LSTM for temporal modeling
        self.lstm = nn.LSTM(hidden_dim, hidden_dim, num_layers=2,
                           batch_first=True)

        # Action decoder
        self.action_decoder = nn.Sequential(
            nn.Linear(hidden_dim, 128),
            nn.ReLU(),
            nn.Linear(128, action_dim),
            nn.Tanh()  # Bounded actions
        )

        # Hidden state for LSTM
        self.hidden = None

    def forward(self, image, proprio, reset_hidden=False):
        """
        Forward pass.

        Args:
            image: Camera image [B, 3, H, W]
            proprio: Proprioceptive state [B, proprio_dim]
            reset_hidden: Whether to reset LSTM hidden state

        Returns:
            action: Predicted action [B, action_dim]
        """
        batch_size = image.size(0)

        # Encode observations
        visual_features = self.visual_encoder(image)
        proprio_features = self.proprio_encoder(proprio)

        # Fuse modalities
        fused = self.fusion(torch.cat([visual_features, proprio_features],
                                      dim=1))

        # Temporal processing
        if reset_hidden or self.hidden is None:
            self.hidden = None

        fused = fused.unsqueeze(1)  # Add sequence dimension
        lstm_out, self.hidden = self.lstm(fused, self.hidden)
        lstm_out = lstm_out.squeeze(1)

        # Decode action
        action = self.action_decoder(lstm_out)

        return action

    def reset_hidden_state(self):
        """Reset LSTM hidden state (call at episode start)."""
        self.hidden = None


class ImitationLearningTrainer:
    """
    Train behavior cloning policy from demonstrations.
    """

    def __init__(self, policy, device='cuda'):
        self.policy = policy.to(device)
        self.device = device

        # Optimizer
        self.optimizer = torch.optim.Adam(policy.parameters(), lr=1e-4)

        # Loss function
        self.criterion = nn.MSELoss()

        # Training history
        self.train_losses = []
        self.val_losses = []

    def load_demonstrations(self, demo_dir):
        """
        Load all demonstrations from directory.
        """
        import os
        import glob

        demo_files = glob.glob(os.path.join(demo_dir, '*.pkl'))

        demonstrations = []
        for file in demo_files:
            with open(file, 'rb') as f:
                demo = pickle.load(f)
                demonstrations.append(demo)

        print(f"Loaded {len(demonstrations)} demonstrations")

        return demonstrations

    def prepare_dataset(self, demonstrations, train_split=0.8):
        """
        Prepare training and validation datasets.
        """
        # Combine all demonstrations
        all_images = []
        all_proprio = []
        all_actions = []

        for demo in demonstrations:
            for obs, action in zip(demo['observations'], demo['actions']):
                # Preprocess image
                image = self.preprocess_image(obs['image'])
                all_images.append(image)

                # Proprioceptive state
                proprio = np.concatenate([
                    obs['joint_positions'],
                    obs['joint_velocities']
                ])
                all_proprio.append(proprio)

                # Action
                all_actions.append(action)

        # Convert to tensors
        images = torch.stack([torch.from_numpy(img) for img in all_images])
        proprio = torch.stack([torch.from_numpy(p) for p in all_proprio])
        actions = torch.stack([torch.from_numpy(a) for a in all_actions])

        # Train/val split
        n_train = int(len(images) * train_split)

        train_dataset = {
            'images': images[:n_train],
            'proprio': proprio[:n_train],
            'actions': actions[:n_train]
        }

        val_dataset = {
            'images': images[n_train:],
            'proprio': proprio[n_train:],
            'actions': actions[n_train:]
        }

        return train_dataset, val_dataset

    def preprocess_image(self, image):
        """Preprocess image for neural network."""
        # Resize to 224x224
        image = cv2.resize(image, (224, 224))

        # Normalize to [0, 1]
        image = image.astype(np.float32) / 255.0

        # Convert to CHW format
        image = np.transpose(image, (2, 0, 1))

        return image

    def train(self, train_data, val_data, epochs=100, batch_size=32):
        """
        Train policy using behavioral cloning.
        """
        n_train = len(train_data['images'])
        n_val = len(val_data['images'])

        for epoch in range(epochs):
            # Training
            self.policy.train()
            epoch_loss = 0.0

            # Shuffle training data
            indices = torch.randperm(n_train)

            for i in range(0, n_train, batch_size):
                batch_indices = indices[i:i+batch_size]

                # Get batch
                images = train_data['images'][batch_indices].to(self.device)
                proprio = train_data['proprio'][batch_indices].to(self.device)
                actions = train_data['actions'][batch_indices].to(self.device)

                # Forward pass
                self.policy.reset_hidden_state()
                pred_actions = self.policy(images, proprio)

                # Compute loss
                loss = self.criterion(pred_actions, actions)

                # Backward pass
                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()

                epoch_loss += loss.item()

            avg_train_loss = epoch_loss / (n_train / batch_size)
            self.train_losses.append(avg_train_loss)

            # Validation
            self.policy.eval()
            val_loss = 0.0

            with torch.no_grad():
                for i in range(0, n_val, batch_size):
                    images = val_data['images'][i:i+batch_size].to(self.device)
                    proprio = val_data['proprio'][i:i+batch_size].to(self.device)
                    actions = val_data['actions'][i:i+batch_size].to(self.device)

                    self.policy.reset_hidden_state()
                    pred_actions = self.policy(images, proprio)

                    loss = self.criterion(pred_actions, actions)
                    val_loss += loss.item()

            avg_val_loss = val_loss / (n_val / batch_size)
            self.val_losses.append(avg_val_loss)

            # Print progress
            if (epoch + 1) % 10 == 0:
                print(f"Epoch {epoch+1}/{epochs}: "
                      f"Train Loss = {avg_train_loss:.6f}, "
                      f"Val Loss = {avg_val_loss:.6f}")

    def save_policy(self, path):
        """Save trained policy."""
        torch.save(self.policy.state_dict(), path)
        print(f"Policy saved to {path}")
```

### 9.5.2 Deployment and Execution

```python
class LearnedBehaviorExecutor:
    """
    Execute learned policies on the robot.
    """

    def __init__(self, policy, robot, camera, device='cuda'):
        self.policy = policy.to(device)
        self.policy.eval()
        self.device = device

        self.robot = robot
        self.camera = camera

    def execute_policy(self, duration=10.0):
        """
        Run learned policy for specified duration.

        Args:
            duration: Execution time in seconds
        """
        print(f"Executing learned policy for {duration}s...")

        self.policy.reset_hidden_state()

        start_time = time.time()

        with torch.no_grad():
            while time.time() - start_time < duration:
                # Get observations
                image = self.camera.read()
                image = self.preprocess_image(image)
                image = torch.from_numpy(image).unsqueeze(0).to(self.device)

                joint_pos = self.robot.get_joint_positions()
                joint_vel = self.robot.get_joint_velocities()
                proprio = np.concatenate([joint_pos, joint_vel])
                proprio = torch.from_numpy(proprio).unsqueeze(0).to(self.device)

                # Predict action
                action = self.policy(image, proprio)
                action = action.cpu().numpy()[0]

                # Execute action (as position delta)
                new_positions = joint_pos + action
                self.robot.set_joint_positions(new_positions)

                # Control rate: 50 Hz
                time.sleep(0.02)

        print("Execution complete")

    def preprocess_image(self, image):
        """Preprocess image (same as training)."""
        image = cv2.resize(image, (224, 224))
        image = image.astype(np.float32) / 255.0
        image = np.transpose(image, (2, 0, 1))
        return image
```

**Performance Expectations**:
- Training time: ~30-60 minutes for 100 epochs on 1000 demonstrations
- Inference rate: ~50 Hz on NVIDIA Jetson Nano
- Success rate: 60-80% for simple manipulation tasks with 10+ demonstrations

<div class="quiz">

**Quiz 9.3: Imitation Learning**

1. What is the "distributional shift" problem in behavioral cloning?
   - a) Data distribution changes over time
   - b) Learned policy visits states not in demonstration data, causing errors to compound
   - c) Different demonstrators have different styles
   - d) Sensors drift during operation

2. Why use an LSTM or Transformer in the policy network?
   - a) Faster computation
   - b) To model temporal dependencies in sequential decisions
   - c) To reduce overfitting
   - d) To handle missing data

3. In the training loss function L = E[(a - π(s))²], what does minimizing this optimize?
   - a) Energy efficiency
   - b) Execution speed
   - c) Matching expert actions given states
   - d) Long-term reward

**Answers**: 1-b, 2-b, 3-c

</div>

---

## 9.6 Reinforcement Learning for Gait Optimization

While imitation learning copies demonstrated behaviors, reinforcement learning discovers optimal policies through trial and error, guided by reward signals.

### 9.6.1 Reinforcement Learning Fundamentals

**Markov Decision Process (MDP)**: RL problems are formalized as MDPs (𝓢, 𝓐, 𝓟, 𝓡, γ):
- 𝓢: State space
- 𝓐: Action space
- 𝓟: Transition dynamics P(s'|s,a)
- 𝓡: Reward function r(s,a)
- γ ∈ [0,1): Discount factor

**Objective**: Find policy π* maximizing expected cumulative reward:

<div class="equation-block">

$$
\pi^* = \arg\max_{\pi} \mathbb{E}_{\tau \sim \pi}\left[\sum_{t=0}^{\infty} \gamma^t r(s_t, a_t)\right]
$$

</div>

**Value Functions**:
- State value: V^π(s) = E_π[Σ γᵗr(sₜ,aₜ) | s₀=s]
- Action value: Q^π(s,a) = E_π[Σ γᵗr(sₜ,aₜ) | s₀=s, a₀=a]

**Bellman Optimality Equation**:

<div class="equation-block">

$$
Q^*(s,a) = \mathbb{E}_{s' \sim P(\cdot|s,a)}\left[r(s,a) + \gamma \max_{a'} Q^*(s', a')\right]
$$

</div>

### 9.6.2 Policy Gradient Methods

For continuous control (like humanoid walking), policy gradient methods directly optimize the policy:

**REINFORCE Algorithm**: Update policy parameters θ in direction of:

<div class="equation-block">

$$
\nabla_\theta J(\theta) = \mathbb{E}_{\tau \sim \pi_\theta}\left[\sum_{t=0}^{T} \nabla_\theta \log \pi_\theta(a_t|s_t) G_t\right]
$$

</div>

where G_t = Σ_{k=t}^T γ^{k-t} r_k is the return from time t.

**Actor-Critic Methods**: Reduce variance by using learned value function as baseline:

<div class="equation-block">

$$
\nabla_\theta J(\theta) \approx \mathbb{E}\left[\nabla_\theta \log \pi_\theta(a_t|s_t) (Q(s_t, a_t) - V(s_t))\right]
$$

</div>

---

## 9.7 Guided Project: Gait Optimization with RL

### Objective
Use reinforcement learning to optimize gait parameters for energy efficiency and stability, discovering gaits that may outperform hand-designed controllers.

### 9.7.1 Implementation: Simple RL for Parameter Optimization

```python
import numpy as np
from collections import deque

class GaitParameterOptimizer:
    """
    Optimize gait parameters using evolutionary strategies or
    simple policy gradient methods.

    State: Current gait parameters
    Action: Parameter adjustments
    Reward: -energy_used + distance_traveled - 100*fall_penalty
    """

    def __init__(self, walker, param_bounds):
        """
        Args:
            walker: Walking controller with get/set parameter methods
            param_bounds: Dict of {param_name: (min, max)}
        """
        self.walker = walker
        self.param_bounds = param_bounds
        self.param_names = list(param_bounds.keys())

        # Best parameters found
        self.best_params = self.walker.get_params()
        self.best_reward = -float('inf')

        # History
        self.reward_history = []

    def evaluate_gait(self, params, n_steps=20):
        """
        Evaluate gait with given parameters.

        Returns:
            reward: Scalar reward value
            metrics: Dict of performance metrics
        """
        # Set parameters
        self.walker.set_params(params)

        # Reset robot
        self.walker.reset_to_standing()

        # Metrics
        total_distance = 0.0
        total_energy = 0.0
        fell = False

        # Execute walking
        for step in range(n_steps):
            # Take step
            success = self.walker.step()

            if not success:
                fell = True
                break

            # Measure distance
            step_distance = self.walker.get_step_distance()
            total_distance += step_distance

            # Measure energy (integral of torque * velocity)
            torques = self.walker.get_joint_torques()
            velocities = self.walker.get_joint_velocities()
            step_energy = np.sum(np.abs(torques * velocities)) * 0.1  # dt = 0.1s
            total_energy += step_energy

        # Compute reward
        distance_reward = total_distance  # cm
        energy_penalty = 0.1 * total_energy  # Weighted penalty
        fall_penalty = 1000.0 if fell else 0.0

        reward = distance_reward - energy_penalty - fall_penalty

        metrics = {
            'distance': total_distance,
            'energy': total_energy,
            'fell': fell,
            'steps_completed': step if fell else n_steps,
            'efficiency': total_distance / (total_energy + 1e-6)
        }

        return reward, metrics

    def optimize_random_search(self, n_iterations=100):
        """
        Simple random search optimization.

        Each iteration:
        1. Sample random parameter variation
        2. Evaluate resulting gait
        3. Keep if better than current best
        """
        print("Starting Random Search Optimization")
        print("=" * 50)

        for iteration in range(n_iterations):
            # Generate random parameters
            test_params = {}
            for param_name in self.param_names:
                min_val, max_val = self.param_bounds[param_name]
                test_params[param_name] = np.random.uniform(min_val, max_val)

            # Evaluate
            reward, metrics = self.evaluate_gait(test_params)
            self.reward_history.append(reward)

            # Update best
            if reward > self.best_reward:
                self.best_reward = reward
                self.best_params = test_params.copy()

                print(f"\nIteration {iteration}: NEW BEST")
                print(f"  Reward: {reward:.2f}")
                print(f"  Distance: {metrics['distance']:.1f} cm")
                print(f"  Energy: {metrics['energy']:.1f}")
                print(f"  Efficiency: {metrics['efficiency']:.3f} cm/J")
                print(f"  Parameters: {test_params}")
            elif iteration % 10 == 0:
                print(f"Iteration {iteration}: reward={reward:.2f}")

        print("\n" + "=" * 50)
        print("Optimization Complete")
        print(f"Best Reward: {self.best_reward:.2f}")
        print(f"Best Parameters: {self.best_params}")

        return self.best_params

    def optimize_cem(self, n_iterations=50, pop_size=20, elite_frac=0.2):
        """
        Cross-Entropy Method (CEM) optimization.

        Maintains a distribution over parameters and iteratively:
        1. Sample population from distribution
        2. Evaluate all samples
        3. Update distribution toward elite samples
        """
        print("Starting CEM Optimization")
        print("=" * 50)

        # Initialize parameter distribution (Gaussian)
        n_params = len(self.param_names)
        mu = np.array([np.mean(self.param_bounds[p])
                      for p in self.param_names])
        sigma = np.array([(self.param_bounds[p][1] - self.param_bounds[p][0]) / 4
                         for p in self.param_names])

        n_elite = int(pop_size * elite_frac)

        for iteration in range(n_iterations):
            # Sample population
            population = []
            for _ in range(pop_size):
                sample = np.random.normal(mu, sigma)

                # Clip to bounds
                for i, param_name in enumerate(self.param_names):
                    min_val, max_val = self.param_bounds[param_name]
                    sample[i] = np.clip(sample[i], min_val, max_val)

                population.append(sample)

            # Evaluate population
            rewards = []
            for params_array in population:
                params_dict = {name: params_array[i]
                              for i, name in enumerate(self.param_names)}
                reward, _ = self.evaluate_gait(params_dict)
                rewards.append(reward)

            # Select elite samples
            elite_indices = np.argsort(rewards)[-n_elite:]
            elite_samples = [population[i] for i in elite_indices]
            elite_rewards = [rewards[i] for i in elite_indices]

            # Update distribution
            mu = np.mean(elite_samples, axis=0)
            sigma = np.std(elite_samples, axis=0) + 1e-6  # Avoid collapse

            # Track best
            best_idx = np.argmax(rewards)
            if rewards[best_idx] > self.best_reward:
                self.best_reward = rewards[best_idx]
                self.best_params = {name: population[best_idx][i]
                                   for i, name in enumerate(self.param_names)}

            # Log progress
            mean_reward = np.mean(elite_rewards)
            print(f"Iteration {iteration}: "
                  f"Mean Elite Reward={mean_reward:.2f}, "
                  f"Best={self.best_reward:.2f}")

        print("\n" + "=" * 50)
        print("CEM Optimization Complete")
        print(f"Best Reward: {self.best_reward:.2f}")
        print(f"Best Parameters: {self.best_params}")

        return self.best_params
```

### 9.7.2 Example Usage

```python
def optimize_humanoid_gait():
    """
    Example: Optimize gait parameters for humanoid walking.
    """
    # Define parameter search space
    param_bounds = {
        'step_length': (3.0, 10.0),      # cm
        'step_height': (1.0, 5.0),       # cm
        'step_time': (0.5, 2.0),         # seconds
        'lateral_sway': (1.0, 5.0),      # cm
        'forward_lean': (-5.0, 10.0),    # degrees
        'hip_amplitude': (10.0, 30.0),   # degrees
    }

    # Initialize optimizer
    optimizer = GaitParameterOptimizer(walker, param_bounds)

    # Run optimization
    # Method 1: Random search (simple, robust)
    best_params_rs = optimizer.optimize_random_search(n_iterations=100)

    # Method 2: CEM (more sample-efficient)
    best_params_cem = optimizer.optimize_cem(n_iterations=50,
                                             pop_size=20)

    # Apply best parameters
    walker.set_params(best_params_cem)

    # Test optimized gait
    print("\nTesting optimized gait...")
    reward, metrics = optimizer.evaluate_gait(best_params_cem, n_steps=50)

    print(f"Final Performance:")
    print(f"  Total Distance: {metrics['distance']:.1f} cm")
    print(f"  Total Energy: {metrics['energy']:.1f} J")
    print(f"  Efficiency: {metrics['efficiency']:.3f} cm/J")
```

**Expected Results**:
- Random search typically finds 10-20% improvement over default parameters
- CEM can achieve 20-30% improvement with fewer evaluations
- Optimized gaits often discover counterintuitive strategies (e.g., slightly asymmetric gait for efficiency)

---

## 9.8 Natural Language Control

Natural language interfaces enable intuitive human-robot interaction through voice commands and linguistic task specification.

### 9.8.1 Speech Recognition and NLP Pipeline

<div class="equation-block">

**Pipeline**: Audio → Transcription → Intent Recognition → Slot Filling → Execution

</div>

**Speech-to-Text**: Convert audio waveform to text using models like Whisper or Google Speech Recognition.

**Intent Classification**: Classify utterance into predefined intents (e.g., "pick_object", "go_to_location", "find_object").

**Slot Filling**: Extract relevant entities (e.g., object="cup", location="table").

### 9.8.2 Implementation: Voice Command System

```python
import speech_recognition as sr
import pyttsx3
from transformers import pipeline

class NaturalLanguageController:
    """
    Natural language interface for humanoid robot control.

    Supports voice commands like:
    - "Pick up the red cup"
    - "Walk to the table"
    - "Find the book"
    - "Give me the bottle"
    """

    def __init__(self, robot):
        self.robot = robot

        # Speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Text-to-speech
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 150)

        # NLP for intent classification
        self.classifier = pipeline(
            "zero-shot-classification",
            model="facebook/bart-large-mnli"
        )

        # Define intents and their handlers
        self.intents = {
            'pick_object': self.handle_pick,
            'place_object': self.handle_place,
            'walk_to': self.handle_walk_to,
            'find_object': self.handle_find,
            'give_object': self.handle_give,
            'stop': self.handle_stop,
            'idle': self.handle_idle
        }

        # Simple keyword-based slot extraction
        self.object_keywords = ['cup', 'bottle', 'book', 'phone',
                               'apple', 'ball']
        self.location_keywords = ['table', 'chair', 'bin', 'shelf']

    def speak(self, text):
        """Text-to-speech output."""
        print(f"Robot: {text}")
        self.tts_engine.say(text)
        self.tts_engine.runAndWait()

    def listen(self, timeout=5):
        """
        Listen for voice command.

        Returns:
            text: Transcribed speech, or None if failed
        """
        with self.microphone as source:
            # Adjust for ambient noise
            self.recognizer.adjust_for_ambient_noise(source, duration=0.5)

            print("Listening...")
            try:
                audio = self.recognizer.listen(source, timeout=timeout)

                # Transcribe
                text = self.recognizer.recognize_google(audio)
                print(f"Heard: \"{text}\"")
                return text

            except sr.WaitTimeoutError:
                print("Listening timeout")
                return None
            except sr.UnknownValueError:
                print("Could not understand audio")
                return None
            except sr.RequestError as e:
                print(f"Speech recognition error: {e}")
                return None

    def classify_intent(self, text):
        """
        Classify user intent from text.

        Uses zero-shot classification to map text to intent.
        """
        # Candidate intents
        candidate_labels = list(self.intents.keys())

        # Classify
        result = self.classifier(text, candidate_labels)

        # Get top intent
        top_intent = result['labels'][0]
        confidence = result['scores'][0]

        return top_intent, confidence

    def extract_slots(self, text):
        """
        Extract entities from text using simple keyword matching.

        For production: use NER (Named Entity Recognition) model.
        """
        text_lower = text.lower()

        slots = {}

        # Extract object
        for obj in self.object_keywords:
            if obj in text_lower:
                slots['object'] = obj
                break

        # Extract location
        for loc in self.location_keywords:
            if loc in text_lower:
                slots['location'] = loc
                break

        # Extract color (simple)
        colors = ['red', 'blue', 'green', 'yellow', 'black', 'white']
        for color in colors:
            if color in text_lower:
                slots['color'] = color
                break

        return slots

    def process_command(self, text):
        """
        Process natural language command.

        Returns:
            success: Whether command was understood and executed
        """
        # Classify intent
        intent, confidence = self.classify_intent(text)

        print(f"Intent: {intent} (confidence: {confidence:.2f})")

        if confidence < 0.5:
            self.speak("I'm not sure what you want me to do")
            return False

        # Extract slots
        slots = self.extract_slots(text)

        print(f"Slots: {slots}")

        # Execute intent
        if intent in self.intents:
            try:
                handler = self.intents[intent]
                return handler(slots)
            except Exception as e:
                self.speak(f"Error executing command: {e}")
                return False
        else:
            self.speak("I don't know how to do that")
            return False

    # Intent handlers

    def handle_pick(self, slots):
        """Handle 'pick up X' command."""
        if 'object' not in slots:
            self.speak("What should I pick up?")
            return False

        obj = slots['object']
        self.speak(f"Picking up the {obj}")

        # Call robot grasp function
        success = self.robot.grasp_object(obj)

        if success:
            self.speak(f"I have the {obj}")
        else:
            self.speak(f"I couldn't find the {obj}")

        return success

    def handle_place(self, slots):
        """Handle 'place X at Y' command."""
        location = slots.get('location', 'table')

        self.speak(f"Placing object at {location}")

        success = self.robot.place_object(location)

        if success:
            self.speak("Done")
        else:
            self.speak("I couldn't place the object")

        return success

    def handle_walk_to(self, slots):
        """Handle 'go to X' command."""
        if 'location' not in slots:
            self.speak("Where should I go?")
            return False

        location = slots['location']
        self.speak(f"Walking to the {location}")

        success = self.robot.navigate_to(location)

        if success:
            self.speak(f"I'm at the {location}")
        else:
            self.speak(f"I couldn't reach the {location}")

        return success

    def handle_find(self, slots):
        """Handle 'find X' command."""
        if 'object' not in slots:
            self.speak("What should I find?")
            return False

        obj = slots['object']
        self.speak(f"Looking for {obj}")

        found = self.robot.search_for_object(obj)

        if found:
            self.speak(f"I found the {obj}")
        else:
            self.speak(f"I couldn't find {obj}")

        return found

    def handle_give(self, slots):
        """Handle 'give me X' command."""
        obj = slots.get('object', 'object')

        self.speak(f"Bringing you the {obj}")

        # Pick up object
        if not self.robot.grasp_object(obj):
            self.speak(f"I couldn't find the {obj}")
            return False

        # Find human
        human_pos = self.robot.detect_human()
        if human_pos is None:
            self.speak("I can't see you")
            return False

        # Walk to human
        self.robot.navigate_to(human_pos)

        # Hand over
        success = self.robot.handoff_to_human()

        if success:
            self.speak("Here you go")
        else:
            self.speak("Handoff failed")

        return success

    def handle_stop(self, slots):
        """Handle 'stop' command."""
        self.speak("Stopping")
        self.robot.stop_all_motion()
        return True

    def handle_idle(self, slots):
        """Handle idle/unclear commands."""
        self.speak("I'm ready for commands")
        return True

    def run_interactive_loop(self):
        """
        Main interactive loop: listen → process → execute.
        """
        self.speak("Hello, I'm ready for commands")

        while True:
            # Listen for command
            command = self.listen(timeout=10)

            if command is None:
                continue

            # Check for exit
            if 'exit' in command.lower() or 'shutdown' in command.lower():
                self.speak("Goodbye")
                break

            # Process command
            self.process_command(command)
```

---

## 9.9 Behavior Trees for Task Coordination

Behavior trees provide a hierarchical framework for coordinating complex behaviors, enabling modular and reactive control.

### 9.9.1 Behavior Tree Architecture

**Node Types**:
1. **Action Nodes**: Execute atomic behaviors (e.g., "grasp object", "walk forward")
2. **Condition Nodes**: Check state predicates (e.g., "object visible?", "battery low?")
3. **Sequence Nodes**: Execute children in order; fail if any child fails
4. **Selector Nodes**: Try children in order; succeed if any child succeeds
5. **Parallel Nodes**: Execute multiple children concurrently

**Execution Semantics**: Each node returns:
- SUCCESS: Task completed
- FAILURE: Task failed
- RUNNING: Task in progress

### 9.9.2 Implementation Example

```python
from enum import Enum

class Status(Enum):
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3

class BehaviorNode:
    """Base class for behavior tree nodes."""

    def tick(self):
        """Execute node logic. Returns Status."""
        raise NotImplementedError

class ActionNode(BehaviorNode):
    """Executes an action."""

    def __init__(self, action_func):
        self.action = action_func

    def tick(self):
        return self.action()

class ConditionNode(BehaviorNode):
    """Checks a condition."""

    def __init__(self, condition_func):
        self.condition = condition_func

    def tick(self):
        return Status.SUCCESS if self.condition() else Status.FAILURE

class SequenceNode(BehaviorNode):
    """Execute children in sequence."""

    def __init__(self, children):
        self.children = children
        self.current_child = 0

    def tick(self):
        while self.current_child < len(self.children):
            status = self.children[self.current_child].tick()

            if status == Status.RUNNING:
                return Status.RUNNING
            elif status == Status.FAILURE:
                self.current_child = 0  # Reset
                return Status.FAILURE
            else:  # SUCCESS
                self.current_child += 1

        # All children succeeded
        self.current_child = 0  # Reset
        return Status.SUCCESS

class SelectorNode(BehaviorNode):
    """Try children until one succeeds."""

    def __init__(self, children):
        self.children = children
        self.current_child = 0

    def tick(self):
        while self.current_child < len(self.children):
            status = self.children[self.current_child].tick()

            if status == Status.RUNNING:
                return Status.RUNNING
            elif status == Status.SUCCESS:
                self.current_child = 0  # Reset
                return Status.SUCCESS
            else:  # FAILURE
                self.current_child += 1

        # All children failed
        self.current_child = 0  # Reset
        return Status.FAILURE

# Example: Service robot behavior tree
def create_service_robot_bt(robot):
    """
    Create behavior tree for service robot:
    1. If battery low, go to charger
    2. Else if object detected on floor, pick it up
    3. Else patrol area
    """

    # Define atomic behaviors
    def check_battery_low():
        return robot.get_battery_level() < 20.0

    def go_to_charger():
        success = robot.navigate_to('charger')
        return Status.SUCCESS if success else Status.FAILURE

    def check_object_on_floor():
        objects = robot.detect_objects_on_floor()
        return len(objects) > 0

    def pick_up_object():
        objects = robot.detect_objects_on_floor()
        if objects:
            success = robot.grasp_object(objects[0])
            return Status.SUCCESS if success else Status.FAILURE
        return Status.FAILURE

    def patrol():
        robot.patrol_area()
        return Status.SUCCESS

    # Build tree
    root = SelectorNode([
        # Priority 1: Charge if low battery
        SequenceNode([
            ConditionNode(check_battery_low),
            ActionNode(go_to_charger)
        ]),

        # Priority 2: Clean up floor
        SequenceNode([
            ConditionNode(check_object_on_floor),
            ActionNode(pick_up_object)
        ]),

        # Priority 3: Patrol
        ActionNode(patrol)
    ])

    return root
```

---

## 9.10 Chapter Summary

### 9.10.1 Key Concepts Mastered

1. **Computer Vision**: YOLO-based object detection and 3D localization enable environmental perception
2. **Imitation Learning**: Behavioral cloning from demonstrations acquires skills without reward engineering
3. **Reinforcement Learning**: Optimization through trial and error discovers optimal policies
4. **Natural Language Interface**: Speech recognition and NLP enable intuitive human-robot interaction
5. **Behavior Coordination**: Behavior trees provide modular, reactive task execution

### 9.10.2 Integration: Autonomous Robot Agent

```python
class AutonomousHumanoidAgent:
    """
    Complete autonomous agent integrating all AI capabilities.
    """

    def __init__(self, robot):
        # Perception
        self.vision = RobotVisionSystem()

        # Learning
        self.policy = BehaviorCloningPolicy()
        self.policy.load_state_dict(torch.load('trained_policy.pth'))

        # Natural language
        self.nlp = NaturalLanguageController(robot)

        # Behavior coordination
        self.behavior_tree = create_service_robot_bt(robot)

        self.robot = robot

    def run(self):
        """Main autonomous operation loop."""

        while True:
            # Listen for commands
            command = self.nlp.listen(timeout=2)

            if command:
                # Execute voice command
                self.nlp.process_command(command)
            else:
                # Autonomous behavior
                status = self.behavior_tree.tick()

                # React to behavior tree result
                if status == Status.FAILURE:
                    self.nlp.speak("Task failed, returning to idle")

            time.sleep(0.1)
```

<div class="quiz">

**Quiz 9.4: Integration**

1. In the autonomous agent architecture, what is the role of the behavior tree?
   - a) Generate natural language responses
   - b) Coordinate high-level task execution and prioritization
   - c) Process camera images
   - d) Learn from demonstrations

2. Why combine multiple AI paradigms (vision, learning, NLP) rather than using one approach?
   - a) Each excels at different aspects (perception, action, interaction)
   - b) It's computationally cheaper
   - c) It's easier to implement
   - d) Historical reasons only

3. What is the benefit of the modular architecture shown in AutonomousHumanoidAgent?
   - a) Faster execution
   - b) Components can be developed, tested, and improved independently
   - c) Lower memory usage
   - d) Simpler code

**Answers**: 1-b, 2-a, 3-b

</div>

---

## 9.11 Advanced Topics and Future Directions

### 9.11.1 Foundation Models for Robotics

Recent work integrates large language models (LLMs) and vision-language models (VLMs) for robotic control:
- Task planning from natural language descriptions
- Zero-shot generalization to novel objects/tasks
- Common-sense reasoning about physical interactions

### 9.11.2 Sim-to-Real Transfer

Training in simulation (e.g., PyBullet, Isaac Gym) then deploying on real hardware:
- Domain randomization to improve robustness
- Reality gap challenges and mitigation strategies
- Hybrid approaches combining simulation and real-world data

### 9.11.3 Lifelong Learning

Continuous improvement from ongoing experience:
- Online adaptation to new environments
- Catastrophic forgetting mitigation
- Meta-learning for rapid task acquisition

---

## References and Further Reading

**Computer Vision**:
1. Redmon, J., et al. (2016). "You Only Look Once: Unified, Real-Time Object Detection." CVPR.
2. Carion, N., et al. (2020). "End-to-End Object Detection with Transformers." ECCV.

**Imitation Learning**:
3. Ross, S., Gordon, G., & Bagnell, D. (2011). "A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning." AISTATS.
4. Chen, L., et al. (2021). "Decision Transformer: Reinforcement Learning via Sequence Modeling." NeurIPS.

**Reinforcement Learning**:
5. Sutton, R. S., & Barto, A. G. (2018). *Reinforcement Learning: An Introduction* (2nd ed.). MIT Press.
6. Schulman, J., et al. (2017). "Proximal Policy Optimization Algorithms." arXiv.

**Robotics and Learning**:
7. Levine, S., et al. (2016). "End-to-End Training of Deep Visuomotor Policies." JMLR.
8. Peng, X. B., et al. (2018). "Sim-to-Real Transfer of Robotic Control with Dynamics Randomization." ICRA.

---

## Exercises

### Conceptual Questions

1. **Explain** the distributional shift problem in behavioral cloning and describe two approaches to mitigate it.

2. **Derive** the policy gradient theorem showing that ∇_θ J(θ) = E[∇_θ log π_θ(a|s) Q^π(s,a)].

3. **Compare** supervised learning from demonstrations vs. reinforcement learning. When is each approach more suitable?

### Computational Exercises

4. **Implement** data augmentation for the imitation learning vision encoder (random crops, color jitter, flips).

5. **Compute** the expected error in depth estimation for an object at 60cm distance, given: focal length uncertainty ±10 pixels, object size uncertainty ±1cm, bounding box noise ±3 pixels.

6. **Analyze** the computational complexity of behavior tree execution. How does tree depth affect worst-case tick time?

### Implementation Projects

7. **Extend** the vision system with object pose estimation using ArUco markers or learned keypoint detection.

8. **Implement** DAgger (Dataset Aggregation) for the imitation learning pipeline, collecting on-policy data to reduce distributional shift.

9. **Create** a behavior tree for a complex task like "clear the table and set it for dinner" with appropriate error handling and recovery behaviors.

10. **Build** a complete autonomous demo that integrates vision, learned manipulation policies, voice commands, and behavior tree coordination to accomplish a multi-step task (e.g., "bring me all the cups from the table").

---

<div class="chapter-footer">

**Next Chapter**: HUM-110 - Final Integration & Showcase
*Complete system integration, optimization, and professional demonstration of the fully autonomous humanoid robot.*

**Previous Chapter**: HUM-108 - Object Manipulation & Coordination
*Whole-body control for simultaneous locomotion and manipulation.*

</div>
