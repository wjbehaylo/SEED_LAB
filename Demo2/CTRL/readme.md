# Localization and Control
This is the code for the localization and control side of the project. This includes integration code between localization and control and computer vision, that interfaces between the rasberry pi and arduino

## files
**arduinoTest.ino**: Arduino code to test that the angle was being received properly.
**Arduino_CV_integration.ino**: Arduino code to receive the calculated angle from the pi.
**move.ino**: Arduino code used for test 1. This includes reciving an angle, rotating, and moving forward 6.5 feet.
**circle.ino**: Arduino code to move the robot in a circle for test 2. This includes reciving an angle, rotating, moving 6 feet, and circling the marker.
