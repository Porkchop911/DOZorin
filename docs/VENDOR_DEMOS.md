# Vendor Demo Packages

This guide documents the vendor demo packages in `vendor/src_ros2/src/`. It is source-derived from the repository snapshot, not live-validated on the robot unless another doc explicitly says so.

Use this with:

- `VENDOR_STACK.md` for the package tour.
- `CODE_WALKTHROUGH.md` for core drivers, Nav2, SLAM, and shared framework details.
- `HARDWARE_INTERACTIONS.md` for hardware-to-topic mapping.
- `OFFLINE_AUDIT.md` for static defects and safety risks.

## Reading Model

The vendor demo layer is mostly made of ROS 2 Python nodes with a common shape:

- A launch file optionally starts prerequisite hardware drivers when `debug=true` or when the demo is launched standalone.
- The node waits for `*/init_finish` services from controller, camera, model, or ASR nodes.
- The node exposes `~/enter`, `~/exit`, `~/start`, `~/stop`, `~/set_running`, or `~/init_finish` services.
- The node subscribes to perception topics such as `/depth_cam/rgb0/image_raw`, `/depth_cam/depth0/image_raw`, `/scan`, `/yolo/object_detect`, or `/asr_node/voice_words`.
- The node publishes hardware commands directly, usually `/controller/cmd_vel`, `/servo_controller`, `servo_controller`, `/ros_robot_controller/pwm_servo/set_state`, or Nav2 goal topics.

The most important safety implication is that many demos bypass Nav2 and publish raw `Twist` commands to `/controller/cmd_vel`. They should be treated as active motion controllers, not passive examples.

## Package Map

| Package | Demo role | Primary hardware touched |
|---|---|---|
| `app` | Phone/app-facing demos and the factory app autostart bundle | Base, lidar, depth/USB camera, arm servos |
| `example` | Teaching demos for color, hands/body, RGB-D grasping, YOLO, self-driving, QR, AR | Camera, depth camera, base, arm, kinematics, TensorRT models |
| `large_models_examples` | LLM/VLM demos that coordinate `large_models`, voice, camera, navigation, sorting, and transport | Base, camera, Nav2, arm, ASR/TTS, model services |
| `openclaw_controller` | Voice and arm/gripper scene demos for OpenClaw workflows | Camera, YOLO, kinematics, servo controller, TTS/ASR gateway |
| `xf_mic_asr_offline` | iFlytek offline wake/ASR and voice-control demos | WonderEcho mic, buzzer, base, arm, color demos, navigation |

## Cross-Cutting Conventions

### Launch Path Selection

Many launch files read `need_compile`:

- `need_compile=True` means use `get_package_share_directory(...)`.
- `need_compile=False` means use hardcoded source paths under `/home/ubuntu/ros2_ws/src/...`.

That matches the robot's symlink-install workflow, where source edits under `~/ros2_ws/src` can take effect after node restart. It also means the launch files are not portable without environment setup.

### Lifecycle Services

Common service names:

| Service | Meaning |
|---|---|
| `~/init_finish` | Node reports that construction and dependency waits are complete. |
| `~/enter` / `~/exit` | Allocate or release subscriptions/resources for a feature. |
| `~/start` / `~/stop` | Begin or stop active behavior after entering the feature. |
| `~/set_running` | Boolean start/stop gate used by many app-style demos. |
| `~/heartbeat` | Vendor watchdog helper that can call `exit` when clients disappear. |

The services are not a full ROS lifecycle implementation. They are vendor-specific gates inside normal `rclpy` nodes.

### Hardware Command Surfaces

| Interface | Effect |
|---|---|
| `/controller/cmd_vel` | Raw base velocity into the vendor controller package. This can move the robot immediately. |
| `/cmd_vel` | Nav2 or teleop-level velocity path in some launches. |
| `/servo_controller` / `servo_controller` | Bus-servo arm/head/gripper commands. Absolute names and relative names both appear. |
| `/ros_robot_controller/pwm_servo/set_state` | PWM servo state command through the low-level controller bridge. |
| `/kinematics/get_current_pose`, `/kinematics/set_pose_target` | Arm pose query/IK target services. |
| `/ros_robot_controller/set_buzzer` | Buzzer feedback through the low-level controller bridge. |

### Perception Topics

| Topic or service | Common role |
|---|---|
| `/depth_cam/rgb0/image_raw` | Primary RGB stream for most camera demos. |
| `/depth_cam/depth0/image_raw` | Depth stream for RGB-D demos. |
| `/usb_cam/image_raw` | Fallback RGB stream in several app/example demos. |
| `/scan` | Lidar scan for obstacle/lidar demos and navigation. |
| `/yolo/start`, `/yolo/stop`, `/yolo/object_detect` | YOLO detector control and detections. |
| `/asr_node/voice_words` | Offline ASR command text. |
| `agent_process/result`, `vocal_detect/wakeup`, `tts_node/play_finish` | Large-model voice/agent event flow. |

## `app`

Source: `vendor/src_ros2/src/app`.

The `app` package is the phone/app-facing demo layer. Its `start_app.launch.py` starts four demo launch files together:

- `lidar_node.launch.py`
- `line_following_node.launch.py`
- `object_tracking_node.launch.py`
- `self_driving_node.launch.py`

This package is not just UI glue. Its demos can command the base and arm directly.

### App Launches

| Launch | Starts | Notes |
|---|---|---|
| `launch/start_app.launch.py` | Includes lidar, line-following, object-tracking, and self-driving launches | Uses `need_compile` path selection. Starts multiple active-capable demos together. |
| `launch/lidar_node.launch.py` | `app/lidar_controller`; optionally lidar and controller launches when `debug=true` | Static issue: calls `get_package_share_directory` but does not import it in this file. |
| `launch/line_following_node.launch.py` | `app/line_following`; optionally lidar, depth camera, and controller when `debug=true` | Static issue: same missing import. |
| `launch/object_tracking_node.launch.py` | `app/object_tracking`; optionally depth camera and controller when `debug=true` | Static issue: same missing import. |
| `launch/self_driving_node.launch.py` | `example/self_driving` plus `example/yolo_node` | Bridges from `app` into the `example` package; does not include depth camera/controller directly in the checked source. |

### App Nodes

| Executable | Source | Purpose | Inputs | Outputs/effects |
|---|---|---|---|---|
| `lidar_controller` | `app/lidar_controller.py` | Lidar-driven base behavior such as avoid/follow/guard-style movement. | Lidar scan and service commands. | Publishes raw base velocity to `/controller/cmd_vel`. |
| `line_following` | `app/line_following.py` | Color/line following using camera image processing. | RGB image, target color, start/running services. | Publishes `/controller/cmd_vel`, `~/image_result`, and servo setup commands. |
| `object_tracking` | `app/object_tracking.py` | Track a colored object and adjust base/servos. | `/depth_cam/rgb0/image_raw` or `/usb_cam/image_raw`; color/threshold services. | Publishes `/controller/cmd_vel`, `~/image_result`, and `servo_controller`. |

### App Packaging Gaps

`app/setup.py` advertises these console scripts that do not have matching source files in `app/app` in this snapshot:

- `ar_app = app.ar_app:main`
- `hand_trajectory = app.hand_trajectory_node:main`
- `hand_gesture = app.hand_gesture:main`

Treat those as stale packaging entries unless live robot validation proves otherwise.

## `example`

Source: `vendor/src_ros2/src/example`.

The `example` package is the broadest teaching/demo package. It contains standalone perception demos, active base demos, active arm demos, and composite demos that chain perception into manipulation or navigation.

### Color Demos

| Demo | Launch/source | Purpose | Hardware interactions |
|---|---|---|---|
| `color_detect` | `example/color_detect/color_detect_node.launch.py`, `color_detect_node.py` | Detect configured colors in camera frames and publish color info. | Subscribes to camera; publishes detection/color result topics; exposes config/start services. |
| `color_track` | `example/color_track/color_track_node.launch.py`, `color_track_node.py` | Track a selected color target. | Camera input; may command servos/base depending mode; exposes color and running services. |
| `color_sorting` | `example/color_sorting/color_sorting_node.launch.py`, `color_sorting_node.py` | Sort objects by color using the arm. | Camera perception plus `servo_controller`/action groups. |
| `garbage_classification` | `example/garbage_classification/garbage_classification.launch.py`, `garbage_classification.py` | Classify garbage categories and trigger sorting behavior. | Camera/model inference and arm/servo actions. |
| `line_follow_clean` | `example/line_follow_clean/line_follow_clean_node.launch.py`, `line_follow_clean_node.py` | Line-following variant for cleanup/path demos. | Camera input and raw base velocity. |

The color demos are reused by voice and large-model demos. For example, voice-control nodes call `/color_detect/*`, `/color_sorting/*`, or `/color_track/*` services instead of duplicating perception.

### Hand, Body, and MediaPipe Demos

| Demo | Launch/source | Purpose | Hardware interactions |
|---|---|---|---|
| `hand_trajectory` | `example/hand_trajectory/hand_trajectory_node.launch.py`, `hand_trajectory_node.py` | Detect hand path and publish trajectory points. | Subscribes to `/depth_cam/rgb0/image_raw`; publishes `~/points` and `~/image_result`; exposes `~/start`, `~/stop`, `~/init_finish`. |
| `hand_gesture_control` | `example/hand_gesture_control/hand_gesture_control_node.launch.py`, `hand_gesture_control_node.py` | Use hand gestures as commands. | Camera input; command effects depend on gesture mapping. |
| `hand_track` / `hand_detect` | `example/hand_track/hand_track_node.py`, `hand_detect_node.py` | Hand detection/tracking examples. | Camera input; image/result outputs. |
| `body_and_rgb_control` | `example/body_control/include/body_and_rgb_control.py` | Body-pose plus RGB feedback/control. | Camera input, RGB/LED or servo effects depending mode. |
| `body_control` | `example/body_control/include/body_control.py` | Body-pose control example. | Camera input; robot control outputs. |
| `body_track` | `example/body_control/include/body_track.py` | Body tracking. | Camera input; potential base/servo tracking outputs. |
| `fall_down_detect` | `example/body_control/include/fall_down_detect.py` | Fall-detection teaching demo. | Camera input and result reporting. |
| `mediapipe_example/*` | `example/mediapipe_example` | Face/hand/pose examples plus arm initialization helper. | Camera input; `include/arm_init.py` publishes `servo_controller` and waits for controller init. |

The package also includes hand model support in `app/app/model`, but the actual runnable hand demos live under `example`, not `app`, in this snapshot.

### Self-Driving and YOLO

| Demo | Launch/source | Purpose | Hardware interactions |
|---|---|---|---|
| `self_driving` | `example/self_driving/self_driving.launch.py`, `self_driving.py` | Lane/traffic-sign demo that follows lanes and reacts to YOLO traffic objects. | Publishes `/controller/cmd_vel`, `/servo_controller`, `/ros_robot_controller/pwm_servo/set_state`, and `~/image_result`; subscribes to camera and `/yolo/object_detect`; calls `/yolo/start` and `/yolo/stop`. |
| `yolo_node` | `example/yolo_detect/yolo_node.py` | Long-running YOLO detector node used by self-driving and other demos. | Subscribes to `/depth_cam/rgb0/image_raw`; publishes `~/object_detect` and `~/object_image`; exposes `/yolo/start`, `/yolo/stop`, and `~/init_finish`. |
| `yolo_detect_demo` | `example/yolo_detect/yolo_detect_demo.launch.py`, `yolo_detect_demo.py` | Standalone configurable YOLO detection demo. | Camera input from `image_topic`; publishes object detections and annotated images. |

Static packaging gap: `example/setup.py` advertises `urban_traffic = example.self_driving.urban_traffic:main`, but `example/self_driving/urban_traffic.py` is not present in this snapshot.

### RGB-D and Arm Function Demos

| Demo | Launch/source | Purpose | Hardware interactions |
|---|---|---|---|
| `cross_bridge` | `example/rgbd_function/cross_bridge.launch.py`, `cross_bridge_node.py` | RGB-D driving/bridge-crossing behavior. | Depth camera and base velocity. |
| `prevent_falling` | `example/rgbd_function/prevent_falling.launch.py`, `prevent_falling_node.py` | Use depth regions to detect drop-offs or plane changes. | Depth image input and base movement policy. |
| `track_object` | `example/rgbd_function/track_object.launch.py`, `track_object_node.py` | Track an object using RGB-D data. | RGB-D input, optional base or servo motion. |
| `object_classification` | `example/rgbd_function/object_classification.launch.py`, `object_classification.py` | Classify objects for manipulation workflows. | Camera/model inference and result topics. |
| `track_and_grab` | `example/rgbd_function/track_and_grab.launch.py`, `track_and_grab.py` | Track a target and grasp it with the arm. | RGB-D camera, kinematics services, `servo_controller`, action groups. |
| `vc_object_classification` | `example/rgbd_function/vc_object_classification.py` | Voice-control wrapper for object classification. | ASR command input plus object-classification services. |
| `vc_track_and_grab` | `example/rgbd_function/vc_track_and_grab.py` | Voice-control wrapper for tracking/grabbing. | ASR command input plus arm/grasp services. |

The `rgbd_function/utils` directory is important support code, not just helpers. It contains calibration, camera offset, plane fitting, grasp yaw calculation, and pick/place routines. These demos should not be run casually: they can move both the chassis and the arm.

### Navigation Transport

| Demo | Launch/source | Purpose | Hardware interactions |
|---|---|---|---|
| `automatic_pick` | `example/navigation_transport/automatic_pick.launch.py`, `automatic_pick.py` | Color/object pickup subroutine for transport demos. | Camera, base alignment, arm action groups, servo controller. |
| `navigation_transport` | `example/navigation_transport/navigation_transport.launch.py`, `navigation_transport.py` | Combine navigation with object pickup/transport. | Nav2 or goal control plus `automatic_pick`/arm actions. |

These are composite demos: navigation moves the base to coarse locations, then vision and arm routines perform close-range manipulation.

### AR, QR, and Utility Demos

| Demo | Source | Purpose | Notes |
|---|---|---|---|
| `ar_detect` | `example/ar_detect/ar_detect.launch.py`, `ar_detect.py` | AR marker detection. | Camera-dependent. |
| QR tools | `example/qrcode/qrcode_creater.py`, `qrcode_detecter.py` | Create and detect QR codes. | Utility scripts, not listed as console scripts in `setup.py`. |
| `color_detect_demo.py` | `example/color_detect/color_detect_demo.py` | Demo/helper around color detection. | Separate from the installed `color_detect` console script. |

## `large_models_examples`

Source: `vendor/src_ros2/src/large_models_examples`.

This package is the demo layer for the vendor `large_models` framework. The underlying framework starts ASR/wakeup, agent, TTS, and VLM/LLM service nodes. The examples translate natural language or visual model results into existing robot demo service calls.

### Shared Large-Model Flow

Most LLM/VLM demos use this chain:

1. `vocal_detect` publishes wake events.
2. `agent_process` receives prompts/model configuration and publishes `agent_process/result`.
3. The demo node parses the result into a robot action.
4. The demo publishes speech text to `tts_node/tts_text`.
5. The node waits for `tts_node/play_finish` or wakeup state before accepting more commands.

Common clients:

- `agent_process/set_model`
- `agent_process/set_prompt`
- `agent_process/set_vllm_content`
- `vocal_detect/enable_wakeup`
- `vocal_detect/set_mode`

### Basic LLM Movement and Perception Demos

| Executable/launch | Purpose | Robot effect |
|---|---|---|
| `llm_control_move` / `llm_control_move.launch.py` | Voice/LLM movement command demo. | Publishes raw `/controller/cmd_vel`; may initialize servos. |
| `llm_control_move_offline` | Offline variant of movement control. | Same movement risk; verify offline mode on robot before relying on it. |
| `llm_color_track` / `llm_color_track.launch.py` | Natural-language color selection for color tracking. | Calls either `/color_track/*` or `object_tracking/*` services depending mode. |
| `llm_visual_patrol` / `llm_visual_patrol.launch.py` | Voice/LLM wrapper around line following/patrol. | Calls `line_following/enter`, `line_following/set_running`, and color target services. |
| `navigation_controller` | Simple pose controller service layer for LLM navigation demos. | Accepts target poses and reports `reach_goal`. |

### VLM Camera and Tracking Demos

| Executable/launch | Purpose | Robot effect |
|---|---|---|
| `vllm_with_camera` | Send camera context to a visual language model and speak/respond. | Camera and large-model services; normally no base motion by itself. |
| `vllm_track` | VLM-selected target tracking. | Camera/depth tracking and possible base motion through tracking helper. |
| `vllm_navigation` | VLM/LLM command to navigation-controller goals. | Uses `navigation_controller/set_pose` and waits for `navigation_controller/reach_goal`. |
| `vllm_track_arm` | VLM target tracking with arm movement. | Uses camera, tracking, arm/servo control. |

Support modules `tracker.py`, `track_anything.py`, and `vllm_track_arm/arm_track_anything.py` implement target tracking and should be treated as active control code.

### Function Calling and Road Network Demos

| Executable/launch | Purpose | Robot effect |
|---|---|---|
| `llm_control` | Function-calling demo entry point. | Dispatches model-selected actions. |
| `road_network_tool` | LLM tool wrapper for road-network commands. | Supplies callable operations to the agent layer. |
| `road_network_navigator` | Navigate along a named road network. | Uses road-network graph plus Nav2 execution. |
| `nav2_execution_node` | Executes Nav2 goals for the road-network layer. | Interfaces with Nav2 action/goal behavior. |
| `road_network/road_network.launch.py` | Starts the road-network navigation stack. | Includes navigation base support and road-network nodes. |

The road-network subtree includes its own launch includes for navigation base behavior. It remaps Nav2 velocity topics in places, so check the launch before combining it with other velocity sources.

### Sorting, Classification, and Transport Demos

| Demo family | Executables/launches | Purpose | Hardware interactions |
|---|---|---|---|
| Color sorting | `llm_object_sorting`, `object_sorting`, `color_sorting/*.launch.py` | Use LLM commands to sort colored objects. | Camera, color detection, arm/servo action. |
| Waste classification | `waste_classification`, `llm_waste_classification` | Classify waste categories and route actions. | Camera/model inference plus arm/sorting behavior. |
| Navigation transport | `automatic_pick`, `vllm_navigation_transport`, `navigation_transport/*.launch.py` | Navigate, identify a box/object, pick/place. | Nav2, RGB-D camera, base alignment, arm action groups. |
| Transport dietitianl | `automatic_transport`, `vllm_transport_dietitianl`, `transport_dietitianl.launch.py` | Diet/transport scene demo; name is vendor spelling. | Navigation, AprilTag/control support, arm transport. |
| Object transport | `object_transport`, `vllm_object_transport` | Object transport workflow. | Camera/VLM, base, arm. |

Several transport demos hardcode `/home/ubuntu/ros2_ws/src/...` config paths and `/home/ubuntu/software/arm_pc/ActionGroups`. Keep those paths in mind when moving the workspace or testing on a PC.

## `openclaw_controller`

Source: `vendor/src_ros2/src/openclaw_controller`.

This package is a higher-level scene controller for OpenClaw-style workflows. It overlaps with `example/rgbd_function` but has its own launch files and voice behavior.

Important caveat: existing docs already note live-image vendor drift for this package. The source snapshot may not exactly match the robot's installed OpenClaw behavior.

### OpenClaw Launches

| Launch | Starts | Purpose |
|---|---|---|
| `launch/start.launch.py` | `voice_openclaw`; optionally `yolo_node.launch.py` when `use_yolo=true` | Main voice OpenClaw entry point. |
| `launch/yolo_node.launch.py` | `example/yolo_node` configured with OpenClaw model choices | Detection support for voice/scenes. |
| `launch/navigation_manager.launch.py` | `track_and_grab` plus navigation-related scene nodes | Navigation plus grab scene. |
| `launch/smart_factory.launch.py` | `track_and_grab` with scene parameters and no YOLO detect by default | Factory-style pick/place scene. |
| `launch/robot_base_control.launch.py` | `track_and_grab` in base-control context | Arm/grab with base-control support. |

### OpenClaw Nodes

| Executable | Source | Purpose | Interfaces |
|---|---|---|---|
| `voice_openclaw` | `openclaw_controller/voice_openclaw.py` | Voice assistant for OpenClaw actions. | Subscribes `/vocal_detect/wakeup`, `/vocal_detect/asr_result`, `/tts_node/play_finish`, `/voice_openclaw/feishu_enable`; publishes `/tts_node/tts_text`; calls `/vocal_detect/enable_wakeup` and `/yolo/start`. |
| `track_and_grab` | `openclaw_controller/track_and_grab_node.py` | Color/object tracking and grasping. | Publishes `/servo_controller`, `~/image_result`, `~/pick_finish`; calls `/kinematics/get_current_pose`, `/kinematics/set_pose_target`, `/controller_manager/init_finish`; exposes `~/start`, `~/stop`, `~/set_color`, `~/init_finish`. |
| `voice_openclaw_simple.py` | `scripts/voice_openclaw_simple.py` | Simple direct ASR script. | Uses local speech library outside the main ROS service chain. |

The OpenClaw demos are arm-active. Run them only when the arm workspace is clear.

## `xf_mic_asr_offline`

Source: `vendor/src_ros2/src/xf_mic_asr_offline`.

This package provides iFlytek offline ASR integration and voice-control demos. It is both C++ and Python:

- `src/voice_control.cpp` wraps the offline recognition engine and publishes recognized words.
- `scripts/awake_node.py` handles wake-word and microphone direction.
- `scripts/asr_node.py` coordinates wake state with recognition requests.
- `scripts/voice_control_*.py` translate recognized words into robot actions.

### Core Voice Launch

| Launch | Starts | Notes |
|---|---|---|
| `launch/mic_init.launch.py` | `awake_node.py`, C++ `voice_control`, and `asr_node.py` for WonderEcho-style mic setup | Exposes wake-word parameters, recording duration, language, and `source_path=/home/ubuntu/ros2_ws/src/xf_mic_asr_offline`. |
| `launch/startup_test.launch.py` | Mic startup test path | Use for hardware bringup/debug, not application behavior. |

Core topics/services:

| Interface | Role |
|---|---|
| `/awake_node/awake_flag` | Wake state consumed by ASR. |
| `/awake_node/angle` | Wake direction angle used by some navigation voice behavior. |
| `/voice_control/get_offline_result` | Service used by `asr_node.py` to request ASR. |
| `/voice_control/voice_words` or `/asr_node/voice_words` | Recognized command text, depending node namespace. |
| `/ros_robot_controller/set_buzzer` | Audio/UX feedback via buzzer. |

### Voice-Control Demo Scripts

| Script/launch | Purpose | Robot effect |
|---|---|---|
| `voice_control_move.py` / `voice_control_move.launch.py` | Direct voice movement. | Publishes `/controller/cmd_vel`; high motion risk. |
| `voice_control_navigation.py` / `voice_control_navigation.launch.py` | Voice navigation and angle-aware movement. | Publishes base velocity and goal poses; uses `/awake_node/angle`. |
| `voice_control_arm.py` / `voice_control_arm.launch.py` | Voice-triggered arm actions. | Runs action groups such as `voice_pick` and `voice_give`; publishes servo commands. |
| `voice_control_color_detect.py` / launch | Voice wrapper for color detection. | Calls `/color_detect/set_param`; subscribes `/color_detect/color_info`. |
| `voice_control_color_sorting.py` / launch | Voice wrapper for color sorting. | Starts/stops color sorting behavior. |
| `voice_control_color_track.py` / launch | Voice wrapper for color tracking. | Starts/stops color tracking and sets color targets. |
| `voice_control_garbage_classification.py` / launch | Voice wrapper for garbage classification. | Calls `/garbage_classification/start` and `/garbage_classification/stop`. |
| `voice_control_navigation_transport.py` / launch | Voice wrapper for transport workflow. | Coordinates navigation transport actions. |
| `wonder_echo_pro_node.py` | Alternative ASR node path for WonderEcho Pro-style output mapping. | Publishes translated ASR results. |

Voice demos are command routers. The risk comes from the downstream feature they trigger: movement, navigation, color tracking, arm action groups, or sorting.

## Known Source-Only Defects and Gaps

These items were found offline and need either source fixes or live validation:

| Finding | Package | Impact |
|---|---|---|
| Missing `get_package_share_directory` import in `app` launch files | `app/launch/lidar_node.launch.py`, `line_following_node.launch.py`, `object_tracking_node.launch.py` | Standalone launch may fail before starting nodes when `need_compile=True`. |
| Stale `app` console scripts | `app/setup.py` | `ar_app`, `hand_trajectory`, and `hand_gesture` entry points do not resolve to files in this snapshot. |
| Stale `example` console script | `example/setup.py` | `urban_traffic` entry point does not resolve to a file in this snapshot. |
| Raw movement spread across demos | `app`, `example`, `large_models_examples`, `xf_mic_asr_offline` | Many demos can command `/controller/cmd_vel` directly. |
| Hardcoded robot paths | Multiple packages | Moving the workspace or running on a PC will break launches/config/action groups unless paths are adapted. |
| Live-image drift | `openclaw_controller` | Published source may not match installed robot behavior. |

## Delegation Slices for a Local Model

These are small tasks that can be offloaded without robot access. Each task should cite source paths and avoid runtime claims.

| Task | Inputs | Output |
|---|---|---|
| Document one `example` family deeply | One subdirectory under `vendor/src_ros2/src/example/example` plus its launch file | A table of topics, services, parameters, and hardware effects. |
| Verify console entry points | `setup.py` plus referenced source files for one package | List of resolvable and stale console scripts. |
| Extract launch dependencies | One package's `launch/*.launch.py` files | Include graph showing which packages/nodes start. |
| Extract raw motion publishers | One package tree | Table of files publishing `/controller/cmd_vel` or `/cmd_vel`. |
| Extract arm/servo users | One package tree | Table of files using `servo_controller`, action groups, or kinematics services. |
| Extract hardcoded paths | One package tree | Table of absolute paths and portability notes. |

Acceptance criteria for any delegated slice:

- Cite every factual statement with a source path.
- Separate source facts from inference.
- Mark missing files or unresolved imports as "not present in this snapshot."
- Do not modify runtime code.
