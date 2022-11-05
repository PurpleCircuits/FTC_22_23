package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auton.PurpleAutoDrive;
import org.firstinspires.ftc.teamcode.auton.PurpleTagRecognition;

@Autonomous(name = "TestAutoLeftBlue", group = "Linear Opmode")
public class TestAutoLeftBlue extends LinearOpMode {

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

    @Override
    public void runOpMode() throws InterruptedException {
        //initalize hardware
        initHardware();
        waitForStart();
        int position = RIGHT;
        //TODO maybe put the like driving to drop cone code before this and only access this later, otherwise repetitive code
        if(purpleTagRecognition.getDetection() == LEFT){
            position = LEFT;
        } else if(purpleTagRecognition.getDetection() == MIDDLE){
            position = MIDDLE;
        } else {
            position = RIGHT;
        }
        if(opModeIsActive())
        purpleAutoDrive.goToPosition(24,0,.75,0,1,5);
        if(opModeIsActive())
        purpleAutoDrive.goToPosition(24,36,.75,0,1,5);
        //Simulate putting claw up
        sleep(2000);
        if(opModeIsActive())
        purpleAutoDrive.goToPosition(28,36,.75,0,1,5);
        if(opModeIsActive())
        purpleAutoDrive.goToPosition(24,36,.75,0,1,5);
        if(position == RIGHT){
            if(opModeIsActive())
            purpleAutoDrive.goToPosition(24,12,.75,0,1,5);
        } else if(position == MIDDLE){
            if(opModeIsActive())
            purpleAutoDrive.goToPosition(24,-12,.75,0,1,5);
        } else {
            if(opModeIsActive())
            purpleAutoDrive.goToPosition(24,-24,.75,0,1,10);
            if(opModeIsActive())
            purpleAutoDrive.goToPosition(36,-24,.75,0,1,5);
        }
        //stops the mapping thread
        purpleAutoDrive.cleanUp();
    }

    private void initHardware() {
        // We are expecting the IMU to be attached to an I2C port (port 0) on a Core Device Interface Module and named "imu".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        purpleTagRecognition = new PurpleTagRecognition();
        purpleTagRecognition.initHardware(hardwareMap);

        purpleAutoDrive = new PurpleAutoDrive();
        purpleAutoDrive.initDriveHardwareMap(hardwareMap);

        // Log that init hardware is finished
        telemetry.log().clear();
        telemetry.log().add("Init. hardware finished.");
        telemetry.clear();
        telemetry.update();
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
