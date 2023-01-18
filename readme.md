https://learnroadrunner.com/#frequently-asked-questions
Driveconstants may have to be changed per robot, set up for the state 23 bot for the Purple Circuits
In grade scripts, instead of looking for the gradle in teamcode, look im the Gradle Scripts folder next to team code, use the build.gradle module teamcode
CURRENTLY we are finishing up 3 wheel localization test (getting us to go the right way on the actual and digital field)

ka and mainly KV need to be edited (like 0.00012 or something, very small numbers)
in StandardTrackingWheelLocalizer on line like 30 something check the forward offset, it needs to be off the center of rotation, i dont really know where that is so i just assumed it was in line with the center bar

right encoder = motorFrontRight
left encoder = motorFrontLeft
front encoder/horizontal encoder = motorBackLeft

may need to reverse some encoders in StandardTrackingWheelLocalizer

OUR FTC Dashboard (in top right dropdown press field to see the field)
http://192.168.43.1:8080/dash