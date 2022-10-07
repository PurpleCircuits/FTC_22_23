package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "TestAutoLeftBlue", group = "Linear Opmode")
public class TestAutoLeftBlue extends LinearOpMode {

    private Trigmecanum trigmecanum = null;
    //private DigitalSensors digitalSensors = null;
    private PurpleTensor purpleTensor = null;


    //Starts recognizing the artifact
    String artifact;

    private static final double SERVO_MIN_POS = 0.0; // Minimum rotational position
    private static final double SERVO_MAX_POS = 1.0; // Maximum rotational position
    private static final double SERVO_OPEN_POS = 0.7; // Start at halfway position


    //private DcMotor theSpinMotor = null;
    //private DcMotor theClawMotor = null;
    //private Servo theClawServo = null;
    private BNO055IMU imu = null;

    private DistanceSensor rightDistance = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //initalize hardware
        initHardware();
        artifact = purpleTensor.getLabel(hardwareMap);
        waitForStart();
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
        moveBotStrafe(12,0,-.5,0);
        //lift slide up
        //forward 10
        moveBotDrive(10,.75,0,0);
        //TODO put cone down
        //back up 10 inches
        moveBotDrive(10,-.75,0,0);
        //put slide down
        if (location == 1){
            //strafe left 36
            moveBotStrafe(36,0,.5,0);
            //forward 12 inches and park
            moveBotDrive(12,.75,0,0);
        }else if (location == 2){
            //strafe left 12
            moveBotStrafe(12,0,.5,0);
            //forward 12 inches and park
            moveBotDrive(12,.75,0,0);
        }else{
            //strafe right 12
            moveBotStrafe(12,0,-.5,0);
            //forward 12 inches and park
            moveBotDrive(12,.75,0,0);
        }
    }

    private void initHardware() {

        trigmecanum = new Trigmecanum();
        trigmecanum.init(hardwareMap, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE);

        //TODO probably re implement digital sensors
        //digitalSensors = new DigitalSensors();
        //digitalSensors.init(hardwareMap);

        purpleTensor = new PurpleTensor();
        // We are expecting the IMU to be attached to an I2C port (port 0) on a Core Device Interface Module and named "imu".

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Log that init hardware is finished
        telemetry.log().clear();
        telemetry.log().add("Init. hardware finished.");
        telemetry.clear();
        telemetry.update();
    }
    private void moveBotStrafe(int inches, double stick1Y, double stick1X, double stick2X){
        String telemetryholder = new String();
        double timeoutS = determineStrafeTime(inches);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            telemetryholder = trigmecanum.mecanumDrive(stick1Y, stick1X, stick2X, false, false);
        }
        trigmecanum.mecanumDrive(0, 0, 0, false, false);
    }
    private void moveBotDrive(int inches, double stick1Y, double stick1X, double stick2X){
        String telemetryholder = new String();
        double timeoutS = determineDriveTime(inches);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            telemetryholder = trigmecanum.mecanumDrive(stick1Y, stick1X, stick2X, false, false);
        }
        trigmecanum.mecanumDrive(0, 0, 0, false, false);
    }
    public void turnLeft(double turnAngle, double timeoutS) {
        if (!opModeIsActive()){
            return;
        }
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed=1;
        double scaledSpeed=speed;
        double targetHeading=angles.firstAngle+turnAngle;
        if(targetHeading<-180) {targetHeading+=360;}
        if(targetHeading>180){targetHeading-=360;}
        double degreesRemaining = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                + (int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < timeoutS && degreesRemaining>3)
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
        if (!opModeIsActive()){
            return;
        }
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed=1;
        double scaledSpeed=speed;
        double targetHeading = angles.firstAngle+turnAngle;
        if(targetHeading < -180) {targetHeading += 360;}
        if(targetHeading > 180){targetHeading -= 360;}
        double degreesRemaining = ((int)(Math.signum(targetHeading-angles.firstAngle)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                + (int)(Math.signum(angles.firstAngle-targetHeading)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeoutS && degreesRemaining>3)
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
    private double determineStrafeTime(int inches){
        double m = 21;
        return inches / m;
    }
    private double determineDriveTime(int inches){
        double m = 30;
        return inches / m;
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
}
