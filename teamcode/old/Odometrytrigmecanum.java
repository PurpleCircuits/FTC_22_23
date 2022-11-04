package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Odometrytrigmecanum {

    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorFrontRight;

    public double motor0Scaled;
    public double motor1Scaled;
    public double motor2Scaled;
    public double motor3Scaled;
    public double slowdown = 0.5;

    public DcMotor encoderLeft;
    public DcMotor encoderRight;
    public DcMotor encoderAux;

    private ElapsedTime runtime = new ElapsedTime() ;

    HardwareMap hwMap;

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    //how many shaft rotations to wheel rotations
    //TODO what is the shaft to wheel ratio? ticks for wheel (383.6 for 40 motor) not needed for what we have?
    double shaftToWheelRatio = 1;
    double ticksPerShaftRotation = 0;
    double ticksPerWheelRotation = shaftToWheelRatio * ticksPerShaftRotation;
    double lrOfset;
    double fbOfset;
    //TODO measure robot again not needed for what we have?
    //robot dimensions
    double robotDiameter = 25.46; //TODO this is if the robot is exactly 18*18 may need to change AGAIN NOT NEEDE FOR WHAT WE HAVE?
    double wheelDiameter = 3.75; //both in inches
    double wheelCircumference = 3.14 * wheelDiameter;

    public void init(HardwareMap ahwMap, DcMotorSimple.Direction frontLeftDirection, DcMotorSimple.Direction frontRightDirection, DcMotorSimple.Direction backLeftDirection, DcMotorSimple.Direction backRightDirection)  {
        hwMap = ahwMap;

        motorFrontLeft = hwMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hwMap.get(DcMotor.class, "motorFrontRight");
        motorBackRight = hwMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft = hwMap.get(DcMotor.class, "motorBackLeft");
        //TODO send in motor direction
        motorFrontLeft.setDirection(frontLeftDirection);
        motorFrontRight.setDirection(frontRightDirection);
        motorBackLeft.setDirection(backLeftDirection);
        motorBackRight.setDirection(backRightDirection);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //set power zero
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        //TODO assign the endcoders to the motors in which they are plugged in
        encoderLeft = motorBackLeft;
        encoderRight = motorBackRight;
        encoderAux = motorFrontRight;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //encoder method, might be needed for reset
        //resetDriveEncoders();
    }
    public String mecanumDrive(double Stick1Y, double Stick1X, double Stick2X, boolean A, boolean Y)  {

        //TODO Stick1X + - values
        double motor0Raw = Stick1Y - Stick1X - (Stick2X);//-Stick1Y - Stick1X - (Stick2X / 2);
        double motor1Raw = -Stick1Y - Stick1X - (Stick2X);//Stick1Y - Stick1X - (Stick2X / 2);
        double motor2Raw = Stick1Y + Stick1X - (Stick2X);//Stick1Y + Stick1X - (Stick2X / 2);
        double motor3Raw = -Stick1Y + Stick1X - (Stick2X);//-Stick1Y + Stick1X - (Stick2X / 2);

        //Find max motor raw values
        double rawMax = Math.max(Math.abs(motor0Raw), Math.max(Math.abs(motor1Raw), Math.max(Math.abs(motor2Raw), Math.abs(motor3Raw))));
        //If any motor power value is outside -1 to 1, scale all values
        if (rawMax > 1) {
            motor0Scaled = motor0Raw / rawMax;
            motor1Scaled = motor1Raw / rawMax;
            motor2Scaled = motor2Raw / rawMax;
            motor3Scaled = motor3Raw / rawMax;
        } else {
            motor0Scaled = motor0Raw;
            motor1Scaled = motor1Raw;

            motor2Scaled = motor2Raw;
            motor3Scaled = motor3Raw;
        }
        if(A) {
            slowdown = 0.5;
        }else if (Y) {
            slowdown = 1;
        }
        motorFrontLeft.setPower(motor0Scaled * slowdown);
        motorFrontRight.setPower(motor1Scaled * slowdown);
        motorBackLeft.setPower(motor2Scaled * slowdown);
        motorBackRight.setPower(motor3Scaled * slowdown);
        double mticks = motorFrontLeft.getCurrentPosition();
        //Outputting wheel telemetry
        String returnData = new String();
        returnData+= "LF" + motorFrontLeft.getCurrentPosition();
        returnData+= "RF" + motorFrontRight.getCurrentPosition();
        returnData+= "LB" + motorBackLeft.getCurrentPosition();
        returnData+= "RB" + motorBackRight.getCurrentPosition();
        return returnData;
    }

    public void fieldOrientedDrive(double Stick1Y, double Stick1X, double Stick2X, boolean A, boolean Y, double imuHeading, boolean rightDpad, boolean upDpad, boolean downDpad) {

        double headingRadians = 0;
        double headingPower = 0;
        //Changes Degrees to Radians
        imuHeading = imuHeading * (Math.PI / 180);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Convert joystick directions to radians and power
        if (Stick1X == 0 && Stick1Y == 0) {
            headingRadians = 0;
            headingPower = 0;
        } else {
            headingRadians = Math.atan2(Stick1X, Stick1Y);
            headingPower = Math.min(Math.sqrt((Stick1X * Stick1X) + (Stick1Y * Stick1Y)), 1);
        }

        //calculate individual motor power
        double motor0Raw = headingPower * -Math.sin(headingRadians + imuHeading + (Math.PI / 4)) - Stick2X * 0.5;
        double motor1Raw = headingPower * -Math.sin(headingRadians + imuHeading + (Math.PI / 4)) - Stick2X * 0.5;
        double motor2Raw = headingPower * -Math.sin(headingRadians + imuHeading + (Math.PI / 4)) - Stick2X * 0.5;
        double motor3Raw = headingPower * -Math.sin(headingRadians + imuHeading + (Math.PI / 4)) - Stick2X * 0.5;

        double rawMax = Math.max(Math.abs(motor0Raw), Math.max(Math.abs(motor1Raw), Math.max(Math.abs(motor2Raw), Math.abs(motor3Raw))));

        //If any motor power value is outside -1 to 1, scale all values
        if (rawMax > 1) {
            motor0Scaled = motor0Raw / rawMax;
            motor1Scaled = motor1Raw / rawMax;
            motor2Scaled = motor2Raw / rawMax;
            motor3Scaled = motor3Raw / rawMax;
        } else {
            motor0Scaled = motor0Raw;
            motor1Scaled = motor1Raw;
            motor2Scaled = motor2Raw;
            motor3Scaled = motor3Raw;
        }

        //If A pressed slow robot
        if (A) {
            slowdown = 0.5;
        } else if (Y) {
            slowdown = 1.0;
        }
        if (rightDpad) {
            lrOfset = 0.25;
        }
        else {
            lrOfset = 0;
        }
        if (upDpad) {
            fbOfset = 0.25;
        }
        else if (downDpad) {
            fbOfset = -0.25;
        }
        else {
            fbOfset = 0;
        }

        motorFrontLeft.setPower(motor0Scaled * slowdown - lrOfset - fbOfset);
        motorFrontRight.setPower(motor1Scaled * slowdown - lrOfset + fbOfset);
        motorBackRight.setPower(motor2Scaled * slowdown + lrOfset + fbOfset);
        motorBackLeft.setPower(motor2Scaled * slowdown + lrOfset - fbOfset);
    }

    //Encoder/Odometry method
    public void resetDriveEncoders(){
        //We might need to make an area where we stop and reset the motor encoders and
        //TODO I dont think this is needed for anything
    }
    public void stop() {
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
    //constants that define the geometry of the robot
    final static double L = 0.0;//distance between encoder 1 and 2 in cm
    final static double B = 0.0;//distance between the midpoint of encoder 1 and 2 and encoder 3
    final static double R = 0.0;//wheel radius in cm
    final static double N = 8192;//encoder ticks per revolution for REV encoders
    final static double cm_per_tick = 2.0 * Math.PI * R / N;

    //tracking odometry encoders between updates
    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentAuxPosition = 0;

    private int oldRightPosition = 0;
    private int oldLeftPosition = 0;
    private int oldAuxPosition = 0;

    //Odometry notes: n1, n2 ,3 are encoder values for the left right and back (aux) omni-wheels
    //dn1 dn2 dn3 are the differences of encoder values between two reads
    //dx dy dtheta describe the robot movement between two reads (in robot coordinates)
    //X Y Theta are the coordinates on the field and the heading of the robot

    //XyhVectir us a tuple where h is the heading of the robot
    //TODO this is an issue, he said there was another video on his channel that covered this
    //public XyhVector START_POS = new XyhVector(213,102, Math.toRadians(-174));
    // public XyhVector pos = new XyhVector(START_POS);

    public void odometry(){

        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldAuxPosition = currentAuxPosition;

        currentRightPosition = -encoderRight.getCurrentPosition();
        currentLeftPosition = -encoderLeft.getCurrentPosition();
        currentAuxPosition = encoderAux.getCurrentPosition();

        int dn1 = currentLeftPosition - oldLeftPosition;
        int dn2 = currentRightPosition - oldRightPosition;
        int dn3 = currentAuxPosition - oldAuxPosition;

        //the robot has moved and turned a tiny bit between two measurements:
        double dtheta = cm_per_tick * (dn2 - dn1) / L;
        double dx = cm_per_tick * (dn1 - dn2) / 2.0;
        double dy = cm_per_tick * (dn3 - (dn2 - dn1) * B / L);

        //small movement of the robot gets added to the field coordinate system:
        //TODO this needs the coordinate system found on lines 239-240ish
        //double theta = pos.h + (Dtheta / 2.0);
        //pos.x += dx * Math.cos(theta) - dy * Math.sin(theta);
        //pos.y += dx * Math.sin(theta) + dy * Math.cos(theta);
        //pos.h += dtheta;

        //limit theta to pos minus Pi or plus minus 180 degrees
        //pos.h = pos.h % (2.0 * Math.PI);
        //if(pos.h > Math.PI) pos.h -= 2.0 * Math.PI;
        //if(pos.h < -Math.PI) pos.h += 2.0 * Math.PI;
    }
}
