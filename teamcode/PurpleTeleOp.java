package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.PurpleOps;
import org.firstinspires.ftc.teamcode.util.Trigmecanum;

@TeleOp(name="PurpelTeleOP", group="Linear Opmode")
public class PurpleTeleOp extends LinearOpMode {
    private Trigmecanum trigmecanum = null;
    private PurpleOps physicaloperation = null;
    private DcMotor theSlideMotor = null;
    BNO055IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        while (opModeIsActive()) {

            trigmecanum.mecanumDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, gamepad1.left_bumper, gamepad1.right_bumper);
            slideAction();
            clawAction();
        }
    }
    private void initHardware(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        trigmecanum = new Trigmecanum();
        trigmecanum.init(hardwareMap, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD);
        //FOR MORE ON INITALIZING MOTORS GO TO TRIGMECANUM

        physicaloperation = new PurpleOps();
        physicaloperation.init(hardwareMap);

        theSlideMotor = hardwareMap.get(DcMotor.class, "the_slide_motor");
        theSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        theSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void clawAction() {
        // close the claw
        if(gamepad2.right_bumper){
            physicaloperation.clawClosed();
        }
        else if(gamepad2.left_bumper){
            physicaloperation.clawOpen();
        }
        //else if (gamepad2.right_bumper || 5 > clawDistance.getDistance(DistanceUnit.CM)) {
        //    theClawServo.setPosition(SERVO_MIN_POS);
        //}
    }
    public void slideAction(){
        double power = -gamepad2.left_stick_y;
        if(gamepad2.b){
            power /= 2;
        }
        //if(gamepad2.right_bumper && theSlideMotor.getCurrentPosition() < 50 && theSlideMotor.getCurrentPosition() > 0){
       //     power = 0;
        //}
        theSlideMotor.setPower(power);
    }
}