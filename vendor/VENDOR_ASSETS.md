# Vendor assets NOT copied into this repo

Everything Hiwonder shipped that is too large or binary to belong in git. Paths are on this PC (`D:\robot`) as of 2026-07-06.

| Asset | Path | Size | What it is |
|---|---|---|---|
| Pristine ROS 2 source zip | `D:\robot\by hiwonder\drive-download-20260624T062759Z-3-003\Source Code\ROS2\src_ros2.zip` | 379.6 MB | Factory zip of the same workspace copied (unzipped) into `vendor/src_ros2/` here |
| STM32 firmware | copied → [`vendor/firmware/RosRobotControllerM4-20260414.hex`](firmware/RosRobotControllerM4-20260414.hex) | 0.18 MB | Compiled firmware for the RosRobotController M4 motor/servo board, build 2026-04-14 |
| System image (multipart archive) | `D:\robot\by hiwonder\...\System Image\ROSOrinPro_Orin_20260509.part01.exe … part09.rar` | ~25 GB compressed | Self-extracting RAR of the full Jetson system disk, dated 2026-05-09 (matches the `VERSION` string the live robot reports) |
| System image (extracted) | `D:\robot\New folder\ROSOrinPro_Orin_20260509.img` | 66.7 GB | Raw disk image ready to flash (Win32DiskImager / DiskGenius) |
| Flashing tools + tutorial | `D:\robot\by hiwonder\...\System Image\System Image Flashing Method & Tool for Jetson Series-...` | 69 MB | `Image System Flashing Tutorials.pdf`, DiskGenius installer, Win32DiskImager |
| Course / manual PDFs | `D:\robot\Jetson Orin Nano & Orin NX-20260623T120657Z-3-001\Jetson Orin Nano & Orin NX\1. Tutorials\` | ~160 MB | 13 course folders. **`1. ROSOrin Pro User Manual.pdf` (94 pp) is the hardware-spec source** used by [`docs/HARDWARE.md`](../docs/HARDWARE.md). Others: chassis, LiDAR, mapping/nav, OpenCV, ML, arm, Gazebo, voice, group control, large AI models, OpenClaw |
| Support software | `...\Jetson Orin Nano & Orin NX\2. Softwares\` | ~700 MB | WonderAi app pointers (Android APK icon / iOS App Store note), VNC Viewer, MobaXterm, SD/disk imaging tools, STM32 serial flashers (ATK-XISP, Flash Loader, FlyMcu), VMware 16 Pro, `PACK_UPDATE_TOOL.exe` (firmware flasher) |
| Assembly videos | `...\Jetson Orin Nano & Orin NX\Assembly\Assembly Video Links.txt` | — | YouTube links; Jetson Orin version: https://youtu.be/b7xMWE_v10Y |
| Dev VM | `D:\robot\ubuntu_ros2_humble-002\` | ~13 GB | VMware VM (Ubuntu 22.04 + ROS 2 Humble, build 2024-12-19) for Gazebo simulation / PC-side development |
| Win32DiskImager | `D:\robot\Win32DiskImager\` and `D:\robot\Win32DiskImager.zip` | 6 MB | Image-flashing tool (also inside the flashing-tool zip above) |
