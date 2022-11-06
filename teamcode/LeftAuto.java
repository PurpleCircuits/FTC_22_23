package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auton.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.auton.PurpleAutoDrive;
import org.firstinspires.ftc.teamcode.auton.PurpleTagRecognition;
import org.firstinspires.ftc.teamcode.util.Trigmecanum;

@Autonomous(name = "LeftAuto")
public class LeftAuto extends LinearOpMode {

    private PurpleTagRecognition purpleTagRecognition = null;
    private PurpleAutoDrive purpleAutoDrive = null;

    // Tag ID 1, 2, and 3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    //private DcMotor theSpinMotor = null;
    //private DcMotor theClawMotor = null;
    //private Servo theClawServo = null;
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
        globalPositionUpdate.reverseRightEncoder();
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
        goToPosition(0*COUNTS_PER_INCH,24*COUNTS_PER_INCH,.75,0,1*COUNTS_PER_INCH);

        goToPosition(36*COUNTS_PER_INCH,24*COUNTS_PER_INCH,.75,0,1*COUNTS_PER_INCH);
        sleep(2000);
        //Simulate putting claw up
        sleep(2000);
        goToPosition(36*COUNTS_PER_INCH,28*COUNTS_PER_INCH,.5,0,1*COUNTS_PER_INCH);
        goToPosition(36*COUNTS_PER_INCH,24*COUNTS_PER_INCH,.5,0,1*COUNTS_PER_INCH);
        //simulate claw down
        sleep(2000);

        if(position == LEFT){
            goToPosition(-24*COUNTS_PER_INCH,24*COUNTS_PER_INCH,.75,0,1*COUNTS_PER_INCH);
            goToPosition(-24*COUNTS_PER_INCH,36*COUNTS_PER_INCH,.75,0,1*COUNTS_PER_INCH);
        } else if(position == MIDDLE){
            goToPosition(0*COUNTS_PER_INCH,24*COUNTS_PER_INCH,.75,0,1*COUNTS_PER_INCH);
            goToPosition(0*COUNTS_PER_INCH,48*COUNTS_PER_INCH,.75,0,1*COUNTS_PER_INCH);
        } else {
            goToPosition(24*COUNTS_PER_INCH,24*COUNTS_PER_INCH,.75,0,1*COUNTS_PER_INCH);
            goToPosition(24*COUNTS_PER_INCH,36*COUNTS_PER_INCH,.75,0,1*COUNTS_PER_INCH);
        }
        //stops the mapping thread
        globalPositionUpdate.stop();
    }


    private void initHardware() {

        purpleTagRecognition = new PurpleTagRecognition();
        purpleTagRecognition.initHardware(hardwareMap);
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

        while(opModeIsActive() && distance > allowableDistanceError) {
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);

            //TODO the x and y may need to be flip flopped, atan2 has been changed since the tutorial? UPDATE, no they have not
            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robot_movement_x_component = calculateX(robotMovementAngle, robotPower);
            double robot_movement_y_component = calculateY(robotMovementAngle, robotPower);
            //double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();
            trigmecanum.mecanumDrive(-robot_movement_y_component, robot_movement_x_component, 0, false, false);
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

    /* OLD CODE FROM LAST YEAR THAT MIGHT BE USEFUL
    private void clawAction(){
        theClawMotor.setTargetPosition(0);
        if(digitalSensors.isCS1AtLimit()){
            theClawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }else{
            while(!digitalSensors.isCS1AtLimit()){
                theClawMotor.setPower(-.25);
            }
            theClawMotor.setPower(0);
        }
    }
    public void rightToDistance(int distance, int timeout){
        runtime.reset();
        while(opModeIsActive() && rightDistance.getDistance(CM) > distance && runtime.seconds() < timeout)
        {
            trigmecanum.mecanumDrive(0,-1,0,false,false);
        }
        trigmecanum.mecanumDrive(0,0,0,false,false);
    }
    private void runToClawPosition(int tics){
        theClawMotor.setTargetPosition(tics);
        theClawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theClawMotor.setPower(.5);
        while (opModeIsActive() && theClawMotor.isBusy()){
            //potential telemetry here if needed
        }
        theClawMotor.setPower(0);
    }
    private void runToColor(double speed, int timeout){
        runtime.reset();
        while(opModeIsActive() && digitalSensors.getColors().blue < .010 && runtime.seconds() < timeout)
        {
            trigmecanum.mecanumDrive(speed,0,0,false,false);
        }
        trigmecanum.mecanumDrive(0,0,0,false,false);
    }
     */
    //TODO OLD AUTO CODE:
    /*

        int location = 0;
        switch (artifact) {
            case "1 Bolt":
                location = 1;
                telemetry.log().clear();
                telemetry.log().add("1 Bolt", "1");
                telemetry.clear();
                telemetry.update();
                sleep(1000);
                break;
            case "3 Panel":
                location = 3;
                telemetry.log().clear();
                telemetry.log().add("3 Panel", "3");
                telemetry.clear();
                telemetry.update();
                sleep(1000);
                break;
            default :
                location = 2;
                telemetry.log().clear();
                telemetry.log().add("2 Bulb", "2");
                telemetry.clear();
                telemetry.update();
                sleep(1000);
                break;
        }
        //TODO Park
        //forward 36 inches
        moveBotDrive(36,.75,0,0);
        //strafe right 12
        moveBotStrafe(12,0,-.75,0);
        //lift slide up
        //forward 10
        moveBotDrive(10,.75,0,0);
        //TODO put cone down
        //back up 10 inches
        moveBotDrive(10,-.75,0,0);
        //put slide down
        if (location == 1){
            //strafe left 36
            moveBotStrafe(36,0,.75,0);
            //forward 12 inches and park
            moveBotDrive(12,.75,0,0);
        }else if (location == 2){
            //strafe left 12
            moveBotStrafe(12,0,.75,0);
            //forward 12 inches and park
            moveBotDrive(12,.75,0,0);
        }else{
            //strafe right 12
            moveBotStrafe(12,0,-.75,0);
            //forward 12 inches and park
            moveBotDrive(12,.75,0,0);
        }
     */
}
