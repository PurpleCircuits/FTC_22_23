package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.auton.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.auton.PurpleAutoDrive;
import org.firstinspires.ftc.teamcode.auton.PurpleTagRecognition;
import org.firstinspires.ftc.teamcode.util.PurpleOps;
import org.firstinspires.ftc.teamcode.util.Trigmecanum;

@Autonomous(name = "LeftAutoMid")
public class LeftAutoMid extends LinearOpMode {

    private PurpleTagRecognition purpleTagRecognition = null;
    private PurpleAutoDrive purpleAutoDrive = null;
    private PurpleOps purpleOps = null;

    private DcMotor theSlideMotor = null;
    private DigitalChannel slideSwitch = null;
    // Tag ID 1, 2, and 3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    private BNO055IMU imu = null;

    //TODO copied from other area
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 1303.83575;

    private Trigmecanum trigmecanum = null;
    private ElapsedTime runtime = new ElapsedTime();

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "motorFrontRight", rbName = "motorBackRight", lfName = "motorFrontLeft", lbName = "motorBackLeft";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;


    @Override
    public void runOpMode() {
        //initalize hardware
        initHardware();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
        //TODO Check lines 71/72 of the sample program to see if these reverses are correct, may have to add these back
        globalPositionUpdate.reverseLeftEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        int position = RIGHT;
        //TODO maybe put the like driving to drop cone code before this and only access this later, otherwise repetitive code
        if(purpleTagRecognition.getDetection() == LEFT){
            position = LEFT;
        } else if(purpleTagRecognition.getDetection() == MIDDLE){
            position = MIDDLE;
        } else {
            position = RIGHT;
        }
        //TODO test code REMOVE BEFORE OFFICIAL TESTING
        purpleOps.clawClosed();
        sleep(500);
        slideAction(3, -.5);

        //goToPosition(0*COUNTS_PER_INCH,26*COUNTS_PER_INCH,.65,0,2*COUNTS_PER_INCH);
        goToPosition(0*COUNTS_PER_INCH,32*COUNTS_PER_INCH,.65,0,2*COUNTS_PER_INCH);
        goToPosition(0*COUNTS_PER_INCH,26*COUNTS_PER_INCH,.65,0,2*COUNTS_PER_INCH);

        goToPosition(13*COUNTS_PER_INCH,26*COUNTS_PER_INCH,.65,0,2*COUNTS_PER_INCH);
        //Did 5 up earlier
        slideAction(22, -.5);

        goToPosition(13*COUNTS_PER_INCH,33*COUNTS_PER_INCH,.5,0,1*COUNTS_PER_INCH);
        slideAction(-3, -.5);
        sleep(500);
        purpleOps.clawOpen();
        sleep(500);
        goToPosition(12*COUNTS_PER_INCH,26*COUNTS_PER_INCH,.65,0,2*COUNTS_PER_INCH);
        //simulate claw down
        //purpleOps.clawClosed();
        slideAction(-22,.5);

        if(position == LEFT){
            goToPosition(-24*COUNTS_PER_INCH,26*COUNTS_PER_INCH,.75,0,2*COUNTS_PER_INCH);
            goToPosition(-24*COUNTS_PER_INCH,36*COUNTS_PER_INCH,.75,0,3*COUNTS_PER_INCH);
        } else if(position == MIDDLE){
            goToPosition(0*COUNTS_PER_INCH,26*COUNTS_PER_INCH,.75,0,2*COUNTS_PER_INCH);
            goToPosition(0*COUNTS_PER_INCH,36*COUNTS_PER_INCH,.75,0,3*COUNTS_PER_INCH);
        } else {
            goToPosition(24*COUNTS_PER_INCH,26*COUNTS_PER_INCH,.75,0,2*COUNTS_PER_INCH);
            goToPosition(24*COUNTS_PER_INCH,36*COUNTS_PER_INCH,.75,0,3*COUNTS_PER_INCH);
        }
        //stops the mapping thread
        globalPositionUpdate.stop();
    }


    private void initHardware() {
        purpleTagRecognition = new PurpleTagRecognition();
        purpleTagRecognition.initHardware(hardwareMap);

        purpleOps = new PurpleOps();
        purpleOps.init(hardwareMap);

        //TODO uncomment these once we have the limit switch
        //slideSwitch1 = hwMap.get(DigitalChannel.class, "slide_switch");
        //slideSwitch1.setMode(DigitalChannel.Mode.INPUT);

        theSlideMotor = hardwareMap.get(DcMotor.class, "the_slide_motor");
        theSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        theSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        trigmecanum = new Trigmecanum();
        trigmecanum.init(hardwareMap, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD);

        // Log that init hardware is finished
        telemetry.log().clear();
        telemetry.log().add("Init. hardware finished.");
        telemetry.clear();
        telemetry.update();
    }

    private void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError){
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        double originalDistance = distance;
        //boolean willTravelMoreThanTenInches = (10*COUNTS_PER_INCH) < originalDistance;

        double orientationWithMath = desiredRobotOrientation - 180;

        while(opModeIsActive() && distance > allowableDistanceError) {
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);

            //TODO the x and y may need to be flip flopped, atan2 has been changed since the tutorial? UPDATE, no they have not
            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            //TODO if distance is less than 10% of original distance (and the original distance was greater than 10 inches(*ticks) - change power to .5 for precision
            //boolean isCloseToPosition = .1 <= (distance/originalDistance);
            //if (willTravelMoreThanTenInches && isCloseToPosition) {
            //robotPower = .5;
            //}
            double robot_movement_x_component = calculateX(robotMovementAngle, robotPower);
            double robot_movement_y_component = calculateY(robotMovementAngle, robotPower);
            double degreeOffAngle = orientationWithMath + globalPositionUpdate.returnOrientation();
            double turnStickPower = 0;
            //if (degreeOffAngle < 0) {
            //Less than zero is negative number, so we must be to the right of the angle
            //turnStickPower = 0.1;
            //} else if (degreeOffAngle > 0){
            //positive number is left of the angle.
            //turnStickPower = -0.1;
            //} else {
            //we are at the angle
            //turnStickPower = 0;
            // }
            trigmecanum.mecanumDrive(robot_movement_y_component, -robot_movement_x_component, 0, false, false);
        }
        trigmecanum.mecanumDrive(0,0,0, false, false);
        //TODO if we move this into another class, get rid of the sleep
        sleep(200);
    }
    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Puts claw (20 motor AndyMark) to specified inch height at a power
     *
     * @param inchHeight the height in inches that you want the slide to go TO
     *
     * @param power specifies the power that you want the motor to run at **NEGATIVE POWER IS UP**
     */
    private void slideAction(double inchHeight, double power){
        double speed = power;
        theSlideMotor.setTargetPosition(0);
        theSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        theSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //38.814 is the ticks per inch on rev motor NOT ANDYMARK
        //537.6 ticks rev for andymark 20 motor / 2 for the gear ratio == 268.8
        //2.3622 inch diameter wheel
        //268.8 / 2.3622pi = 36.221191011 ticks per inc (145.101) 36.2753*2 = 78.5506
        theSlideMotor.setTargetPosition((int)(inchHeight * 75));
        theSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theSlideMotor.setPower(speed);
        while(opModeIsActive() && theSlideMotor.isBusy()){
        }
        theSlideMotor.setPower(0);
    }

    public void turnLeft(double turnAngle, double timeoutS) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed=1;
        double scaledSpeed=speed;
        double targetHeading=angles.firstAngle+turnAngle;
        if(targetHeading<-180) {targetHeading+=360;}
        if(targetHeading>180){targetHeading-=360;}
        double degreesRemaining = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                + (int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        runtime.reset();
        while(runtime.seconds() < timeoutS && degreesRemaining>3)
        {
            //Change the 10 on the line below to a variable
            scaledSpeed = degreesRemaining / (10 + degreesRemaining) * speed;
            if(scaledSpeed>1 || scaledSpeed<.5){scaledSpeed=.5;}//We have a minimum and maximum scaled speed

            trigmecanum.mecanumDrive(0,0, scaledSpeed, false, false);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            degreesRemaining = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                    + (int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        }
        trigmecanum.mecanumDrive(0, 0, 0, false, false);
    }
    public void turnRight(double turnAngle, double timeoutS) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed=1;
        double scaledSpeed=speed;
        double targetHeading = angles.firstAngle+turnAngle;
        if(targetHeading < -180) {targetHeading += 360;}
        if(targetHeading > 180){targetHeading -= 360;}
        double degreesRemaining = ((int)(Math.signum(targetHeading-angles.firstAngle)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                + (int)(Math.signum(angles.firstAngle-targetHeading)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        runtime.reset();
        while (runtime.seconds() < timeoutS && degreesRemaining>3)
        {
            scaledSpeed=degreesRemaining/(10+degreesRemaining)*speed;
            if(scaledSpeed>1 || scaledSpeed<.5){scaledSpeed=.5;}//We have a minimum and maximum scaled speed

            trigmecanum.mecanumDrive(0,0, -scaledSpeed, false, false);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            degreesRemaining = ((int)(Math.signum(targetHeading-angles.firstAngle)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                    + (int)(Math.signum(angles.firstAngle-targetHeading)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        }
        trigmecanum.mecanumDrive(0, 0, 0, false, false);
    }

}
