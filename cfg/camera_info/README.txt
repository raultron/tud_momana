Calibration files for the wireless RunCam 2
The video feed is transmitted by RF in PAL standard
The image comes interlaced

To calibrate:
roslaunch tud_momana jus_camera.launch
rosrun camera_calibration cameracheck.py monocular:=/cam --size 9x7 --square 0.1


Command to check the calibration in our checkerboard:
rosrun camera_calibration cameracheck.py monocular:=/cam --size 9x7 --square 0.1

Distance from camera to checkerboard = 0.95 m (not ideal distance for checkerboard, but is the best distance for the aruco markers and two robots).

Calibration File  |  LRMS  (pixels) | RRMS (pixels) | Aruco distance between markers (0.72m)
cam_v0.1.yaml     |  0.172          | 0.12          |   0.727
cam_v0.2.yaml     |  0.236          | 0.20          |   0.725  *best overall behaviour, also best LRMS when checkerboard is at 1.5 meters
cam_v0.3.yaml     |  1.150          | 1.30	    |   0.712
cam_v0.4.yaml     |  1.044          | 0.73          |   0.736

