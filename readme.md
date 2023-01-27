https://learnroadrunner.com/#frequently-asked-questions
Driveconstants may have to be changed per robot, set up for the state 23 bot for the Purple Circuits
In grade scripts, instead of looking for the gradle in teamcode, look im the Gradle Scripts folder next to team code, use the build.gradle module teamcode
CURRENTLY We are in the feedforward tuning process changing the kV and kA, the robots just not working right now

ka and mainly KV need to be edited (like 0.00012 or something, very small numbers)
in StandardTrackingWheelLocalizer on line like 30 something check the forward offset, it needs to be off the center of rotation, i dont really know where that is so i just assumed it was in line with the center bar


ROADRUNNER ENCODER DEFINITIONS
right encoder = motorFrontRight
left encoder = motorFrontLeft
front encoder/horizontal encoder = motorBackLeft

----------------------------------------------------------------------------

OUR ENCODER DEFINITIONS:
String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;

frontleft   = 0 rightencoder
frontright  = 1 horizontal
backleft    = 2
backright   = 3 leftencoder
may need to reverse some encoders in StandardTrackingWheelLocalizer

OUR FTC Dashboard (in top right dropdown press field to see the field)
http://192.168.43.1:8080/dash

Driveconstants kA kV and kStatic can be changed in the dashboard *TEMPORARILY* but permenantly in the DriveConstants class line
**PRESS ENTER TO OFFICIALLY CHANGE IT**

CHANGE the divisor on pivotCorrection to like 100 or someting, its not creating a substantial number yet, 
MAYBE say if its over 1 set it to 1 as to not go over 