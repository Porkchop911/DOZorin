# Understanding This Robot — A Beginner's Guide

This guide assumes you have **never used a robot, ROS, or Linux command line before**.
It explains the jargon as it goes. Read it top to bottom the first time; later you can
jump to the section you need. There is a plain operator's cheat-sheet in `OPERATING.md`
once these ideas make sense.

---

## Part 0 — What is this thing?

You have a small wheeled robot (the "**ROSOrin Mecanum Pro**") with a camera, a laser
scanner, and a robot arm with a gripper ("claw"). Its brain is a small computer called an
**NVIDIA Jetson** — think of it as a Raspberry Pi on steroids, good at AI and cameras.

The Jetson runs **Linux** (specifically Ubuntu). Linux is just an operating system, like
Windows or macOS, but it's usually driven by **typing commands into a terminal** instead of
clicking. A **terminal** (or "shell") is the black text window where you type commands. The
program reading your commands here is called **zsh** — that's just one brand of shell.

The robot's behaviour is controlled by software written using **ROS 2**.

### What is ROS 2?

**ROS** stands for **Robot Operating System**. The name is misleading — it is *not* an
operating system. It's a **toolbox and a set of rules that let many small programs talk to
each other**. The "2" is the version (we're on a release nicknamed **Humble**).

The core idea of ROS: instead of one giant program that does everything, you have **lots of
tiny programs**, each doing one job, all chatting over a shared messaging system. This is
nice because if one part breaks, you can restart just that part.

Here is the vocabulary you'll see everywhere. Learn these five words and ROS stops being scary:

| Word | Plain meaning | Analogy |
|---|---|---|
| **Node** | One small program doing one job (e.g. "read the camera"). | One employee with one task. |
| **Topic** | A named channel that messages flow through. | A radio station / a group chat. |
| **Message** | A single piece of data sent on a topic. | One text in the chat. |
| **Publish** | To send a message onto a topic. | Speaking into the radio. |
| **Subscribe** | To listen to a topic and react. | Tuning into the radio station. |

So: a **node** *publishes* **messages** on a **topic**, and other **nodes** *subscribe* to
that topic to receive them. Nobody needs to know who else is listening. The camera node just
shouts pictures onto the `/camera` topic; anyone who cares listens.

Two more, slightly more advanced:

- **Service** — like a topic, but it's a *question-and-answer* instead of a broadcast. You
  ask, you wait, you get one reply. (A topic is "radio"; a service is "phone call".)
- **Parameter** — a setting for a node, set when it starts (e.g. "max speed = 0.2").

---

## Part 1 — How this robot is organised (the "stack")

People call the whole pile of software a **"stack"** because it's built in layers, like a
cake. High-level "smart" stuff sits on top; low-level "move the motor" stuff sits at the
bottom. Your **commands travel down** the layers; **sensor readings travel up**.

```
   YOU (web page, joystick, voice, keyboard)        ← "I want to go forward"
            |
   Navigation / apps / AI                           ← decide HOW to move
            |
   Motion controller   (turns "go forward" into wheel speeds)
            |
   Serial bridge       (translates into the robot's private hardware language)
            |
   STM32 microcontroller chip  → spins motors, reads sensors   ← the muscles
```

- **STM32** is a tiny, cheap chip (a "**microcontroller**") that directly controls motors and
  reads sensors. It is *not* the Jetson; it's a helper chip the Jetson talks to.
- The Jetson and the STM32 talk over a **serial port** — a simple wire-based way for two
  chips to exchange bytes one after another. On this robot that connection shows up as a
  device file named **`/dev/rrc`**. (In Linux, hardware appears as files under `/dev/`.)
- **Baud rate** (you'll see "1 Mbaud") just means *how fast* those bytes travel — 1 million
  signal-changes per second here.

### "Mecanum" — why this robot can slide sideways

Most toy cars can only go forward/back and turn. This robot has **mecanum wheels**: special
wheels with little angled rollers around the rim. By spinning the four wheels in different
combinations, the robot can drive **forwards, sideways (strafe), diagonally, and spin in
place** — like a crab. Software that mixes the four wheel speeds to achieve this is called
the **kinematics** (the maths of motion).

---

## Part 2 — Turning the robot on and off

The robot software **starts automatically when the Jetson boots up**. This is arranged by
**systemd**, the part of Linux that runs background programs (called **services** or
**daemons**) and keeps them alive.

The service here is called `start_app_node.service`. Useful commands (type them in a terminal):

```bash
systemctl status start_app_node.service
```
This *checks* whether the robot software is running. (`systemctl` = "control systemd".)

```bash
sudo systemctl restart start_app_node.service
```
This **restarts** everything — the easiest fix when the robot acts weird. `sudo` means "do
this as the administrator" (Linux asks for extra permission for powerful actions; it may ask
for a password).

```bash
sudo systemctl stop start_app_node.service
```
This **stops** the robot software. Note: it's set to auto-revive itself after 30 seconds, so
to keep it off you also `disable` it:

```bash
sudo systemctl disable start_app_node.service
```

**Panic button.** If the robot misbehaves and you just want everything to halt, there's a
ready-made script in the home folder:

```bash
~/.stop_ros.sh
```
The `~` means "my home folder" (`/home/ubuntu`). This script forcibly kills every robot
program. (It uses `kill -9`, which is the "stop immediately, no questions" command.)

---

## Part 3 — "Launching" things and what `bringup` means

Starting one node at a time would be tedious — this robot runs ~25 of them. ROS solves this
with **launch files**: a recipe that starts many nodes at once with the right settings. You
run one with:

```bash
ros2 launch <package> <launchfile>
```

- `ros2` is the main command-line tool for everything ROS.
- A **package** is a folder of related nodes (e.g. the `navigation` package). Think "app".
- The big master launch file here is called **`bringup`** — "bringing the robot up", i.e.
  waking up all the basic systems (motors, camera, laser, web tools). It's what the
  auto-start service runs:

```bash
ros2 launch bringup bringup.launch.py
```

> ⚠️ **Important beginner trap:** the robot is *already running this* from boot. If you run
> it again by hand you'll have **two copies fighting over the same hardware wire** (`/dev/rrc`)
> and nothing will work. Always `stop` the service first (Part 2) before launching by hand.

### Making `ros2` commands work in a new terminal: "sourcing"

If you open a fresh terminal and type `ros2`, it might say "command not found". That's because
ROS isn't loaded into that terminal yet. Loading it is called **"sourcing a setup file"** —
running a script that sets up your terminal's environment. Type:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

("**source**" means "run this script and keep its effects in my current terminal.") You do
this once per new terminal. The auto-start service does it for you automatically.

---

## Part 4 — Looking at what the robot is doing (read-only, totally safe)

These commands only *observe*. They can't break anything — great for learning.

```bash
ros2 node list
```
Lists every running node (program). You'll see names like `/yolo_node`, `/ekf_filter_node`.

```bash
ros2 topic list
```
Lists every channel of data. Names start with `/`. Examples you'll see:
- `/odom` — the robot's best guess of **where it is** (see "odometry" below).
- `/scan` — distances measured by the **laser scanner** (LiDAR).
- `/imu` — tilt/rotation from the **IMU** (a motion-sensing chip: accelerometer + gyroscope).
- `/ros_robot_controller/battery` — battery voltage.

```bash
ros2 topic echo /odom
```
**Prints out the live messages** flowing on a topic so you can watch them. Press `Ctrl-C`
(hold Control, tap C) to stop — `Ctrl-C` is the universal "stop this command" in a terminal.

```bash
ros2 topic hz /scan
```
Shows *how many times per second* messages arrive (`hz` = hertz = times per second). Handy to
check a sensor is alive.

### Jargon this section introduced

- **Odometry** — estimating *how far you've moved* by counting wheel turns and sensing motion.
  Like closing your eyes and guessing your position by counting your steps. It drifts over
  time (small errors add up), which is why robots also use cameras/lasers to correct.
- **LiDAR** — a spinning laser that measures distance to walls in every direction. Produces a
  "**scan**". It's how the robot "sees" obstacles and builds maps.
- **IMU** — Inertial Measurement Unit. A chip that feels acceleration and rotation (the same
  kind of sensor that knows when you rotate your phone).
- **EKF** (you'll see `ekf_filter_node`) — Extended Kalman Filter. A clever bit of maths that
  **blends** the wheel odometry and the IMU into one smoother, more accurate position guess.
  You don't need the maths; just know "EKF = sensor-blender for better position".
- **TF** — short for "transforms". The robot must know where each part is relative to the
  others (where the camera is vs. the wheels vs. the map). TF is the bookkeeping system that
  tracks all these positions. Run `ros2 run tf2_tools view_frames` to get a diagram.

---

## Part 5 — Driving the robot yourself

Movement commands are sent as a message type called **`Twist`**. A Twist describes a velocity
in two parts:
- **linear** — straight-line speed in metres/second. `x` = forward/back, `y` = left/right
  (sideways, because mecanum!), `z` = up/down (unused on the ground).
- **angular** — turning speed in radians/second. `z` = spinning left/right.

(A **radian** is just a way to measure angles; ~6.28 radians = one full turn.)

The robot listens for Twist messages on a few topics. The **safe** one is `/cmd_vel`
("command velocity"), which is automatically limited to a gentle speed:

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'
```
`ros2 topic pub` = "publish a message". `-1` means "just once". This nudges it forward at
0.1 m/s. **To stop it**, send an all-zero Twist:

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{}'
```

> 🛑 **Safety first:** put the robot on a stand or clear floor before sending move commands,
> and always have the stop command ready. Start tiny (0.05–0.1).

**Easier ways to drive:**

```bash
ros2 launch peripherals teleop_key_control.launch.py   # drive with keyboard keys
```
A **joystick/gamepad** also already works (it's started by `bringup`). "**Teleop**" just means
"tele-operation" = driving it remotely/by hand, as opposed to it driving itself.

---

## Part 6 — Watching through the robot's eyes (the web tools)

When `bringup` runs, it also starts two helpers that let a web browser talk to the robot:

- **web_video_server** — puts the camera feeds on a web page. On another computer on the same
  network, open `http://<robot-ip>:8080` (replace `<robot-ip>` with the robot's address; the
  `:8080` is the "**port**", like a specific door into the computer). You'll see the live
  camera, plus processed views like object detection boxes.
- **rosbridge** (port 9090) — a translator that lets web pages send/receive ROS messages. The
  phone/tablet app uses this behind the scenes.

> ⚠️ **Security note for beginners:** these have **no password**. Anyone on the same Wi-Fi can
> open the camera or even drive the robot. Only use it on a network you trust (your home/lab
> Wi-Fi), not open public Wi-Fi.

---

## Part 7 — The robot driving itself: maps and navigation

Two big skills, and they're separate steps:

### SLAM — making a map
**SLAM** = "Simultaneous Localization And Mapping". Long words, simple idea: as the robot
drives around an unknown room, it uses the laser scanner to **draw a map** *and* figure out
**where it is on that map** at the same time. Start it with:

```bash
ros2 launch slam slam.launch.py
```
Then drive the robot slowly around the whole room (Part 5). When the map looks complete, it
gets saved into `~/ros2_ws/src/slam/maps/`. There are already saved maps named `map_01`,
`map_02`, `map_03`.

### Navigation — using a map to go places by itself
Once a map exists, **navigation** lets the robot plan a route and drive to a goal on its own,
avoiding obstacles. The popular ROS system for this is nicknamed **Nav2**.

```bash
ros2 launch navigation navigation.launch.py map:=map_01
```
`map:=map_01` tells it which saved map to use (note: no `.yaml` ending). To then tell it
*where to go*, the easiest way is a visualization program called **RViz** (a 3D viewer for
robot data — needs a screen attached):

```bash
ros2 launch navigation rviz_navigation.launch.py
```
In RViz you click a "2D Goal Pose" button and click on the map; the robot drives there.

Jargon: a **costmap** is the robot's internal map coloured by "how dangerous is it to be
here" (walls = expensive, open floor = cheap); a **planner** finds the cheapest route through
it. **TEB** is the name of one such planner this robot can use.

---

## Part 8 — The "smart" apps (camera & AI behaviours)

These come pre-loaded (started by `bringup`) but sit idle until you switch them on (usually
from the app). Quick tour:

- **line following** — the robot follows a coloured line on the floor using the camera.
- **object tracking** — it keeps a chosen coloured object centred in view, turning to follow it.
- **self driving** — a demo of lane/sign-style autonomous driving.
- **YOLO** — the name of a popular **AI object-detection** model. "**YOLO**" = "You Only Look
  Once" (the technique's name). The `yolo_node` looks at the camera and draws boxes around
  things it recognises, publishing the results on `/yolo/object_detect`.

### The AI / voice / arm features ("large models", "openclaw")
- **LLM** = "Large Language Model" (the tech behind chatbots like Claude). **VLM/vLLM** =
  the same idea but it can also *see images*. This robot has demos where you **speak a command**
  and an AI decides how the robot should move or what to grab.
- **openclaw** is the name for the robot's **gripper arm** software. There are launch files to
  make the arm track and grab objects, or respond to voice. These talk to a small AI "gateway"
  program running locally on the robot.

You generally trigger these from the app rather than by hand at first.

---

## Part 9 — Changing the software (only when you're ready)

The robot's code lives in `~/ros2_ws/src/`, organised into package folders. Most of it is
written in **Python** (a beginner-friendly programming language). If you edit a Python file,
you usually need to **restart** the affected program for the change to take effect:

```bash
sudo systemctl restart start_app_node.service
```

Some changes (especially anything in C++, or changing message definitions) require
**rebuilding** first — "building" means turning source code into a runnable program. The tool
for that here is **colcon**:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```
(`cd` = "change directory" = move into a folder.) Don't worry about this until you actually
need to change code — and **back up a file before you edit it** (`cp file.py file.py.bak`).

---

## Part 10 — Known quirks (so they don't confuse you)

Every real robot has rough edges. These are this one's, in plain terms:

1. **Sideways driving confuses its sense of position.** When the robot strafes sideways, its
   odometry (Part 4) doesn't account for it properly, so its position guess drifts. If a map
   gets messed up, this is often why.
2. **The topic names are misleading.** `/cmd_vel` is actually the *speed-limited, safe* one;
   `/controller/cmd_vel` is the *full-speed, unlimited* one. Beginners should stick to
   `/cmd_vel`.
3. **A couple of features are broken in this build** — sending to `/set_odom` can crash a
   program, and one calibration/servo-reading function errors out. Just avoid them.
4. **Never run two copies of the robot software at once** (Part 3) — they fight over the
   hardware.
5. **No passwords on the web tools** (Part 6) — stay on a trusted network.

---

## Glossary (quick reference)

- **Node** — one small program doing one job.
- **Topic** — a named channel messages flow on.
- **Message** — one piece of data on a topic.
- **Publish / Subscribe** — send onto / listen to a topic.
- **Service** — a request→reply (vs. a topic's broadcast).
- **Parameter** — a start-up setting for a node.
- **Package** — a folder of related nodes (an "app").
- **Launch file** — a recipe that starts many nodes at once.
- **bringup** — the master launch file that wakes the whole robot.
- **Sourcing** — loading ROS into your terminal so `ros2` works.
- **Terminal / shell / zsh** — the text window you type commands into.
- **sudo** — run a command as administrator.
- **Ctrl-C** — stop the currently running command.
- **systemd / service** — Linux's manager for always-on background programs.
- **Serial port / `/dev/rrc` / baud** — the wire and speed the Jetson uses to talk to the STM32 chip.
- **STM32 / microcontroller** — the small chip that directly drives motors & reads sensors.
- **Jetson** — the main AI computer (the brain).
- **Mecanum wheels** — special wheels that let the robot slide sideways.
- **Kinematics** — the maths converting desired motion into wheel speeds.
- **Twist** — the message type for "move at this velocity".
- **linear / angular** — straight-line speed / turning speed.
- **Teleop** — driving the robot by hand (keyboard/joystick).
- **Odometry / `/odom`** — position estimated from motion.
- **IMU** — motion-sensing chip (accelerometer + gyroscope).
- **LiDAR / scan** — spinning laser that measures distances.
- **EKF** — maths that blends sensors into a better position estimate.
- **TF** — system tracking where each robot part is relative to others.
- **SLAM** — build a map while figuring out where you are on it.
- **Navigation / Nav2** — plan and drive a route to a goal automatically.
- **Costmap / planner / TEB** — danger-map / route-finder used by navigation.
- **YOLO** — an AI model that detects objects in camera images.
- **LLM / VLM** — AI language model / language-and-vision model.
- **openclaw** — this robot's gripper-arm software.
- **colcon** — the tool that builds (compiles) the code.
