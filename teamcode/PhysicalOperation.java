package org.firstinspires.ftc.teamcode;



import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PhysicalOperation {
    private static final double SERVO_MIN_POS = 0.25; // Minimum rotational position
    private static final double SERVO_MAX_POS = 1; // Maximum rotational position
    private static final double SERVO_OPEN_POS = 0.7; // Start at halfway position
    private DcMotor theSlideMotor = null;
    private Servo theClawServo = null;
    HardwareMap hwMap;

    public void init(HardwareMap hardwareMap){
        hwMap = hardwareMap;
        theClawServo = hardwareMap.get(Servo.class, "the_claw_servo");

        theSlideMotor = hardwareMap.get(DcMotor.class, "the_slide_motor");
        theSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        theSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void clawOpen(){
        theClawServo.setPosition(SERVO_OPEN_POS);
    }
    public void clawClosed(){
        theClawServo.setPosition(SERVO_MIN_POS);
    }
   }
