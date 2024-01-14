package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helper.MecanumDrive;
import org.firstinspires.ftc.teamcode.Helper.ServoMechanism;

@TeleOp(name="Fighting Cougars Main TeleOp", group="Tests")
public class MecanumDriveTest extends LinearOpMode {

    public static final String RIGHT_FRONT_MOTOR_NAME = "motorFR";
    public static final String LEFT_FRONT_MOTOR_NAME = "motorFL";
    public static final String RIGHT_REAR_MOTOR_NAME = "motorBR";
    public static final String LEFT_REAR_MOTOR_NAME = "motorBL";
    public static final String LINEAR_SLIDE_MOTOR_NAME = "slide";
    public static final String IMU_NAME = "imu";
    public static final double ENCODER_RESOLUTION = 3895.9; // Change accordingly
    public static final double WHEEL_DIAMETER_CM = 9.6; // Change accordingly
    public static final int SLIDE_LENGTH_ENCODER_TICKS = 3845; // Change accordingly
    public int MINIMUM_SLIDE_POSITION; // Change accordingly
    public int MAXIMUM_SLIDE_POSITION; // Change accordingly

    @Override
    public void runOpMode() {

        MecanumDrive mecanum = new MecanumDrive(
                hardwareMap,
                RIGHT_FRONT_MOTOR_NAME,
                LEFT_FRONT_MOTOR_NAME,
                RIGHT_REAR_MOTOR_NAME,
                LEFT_REAR_MOTOR_NAME,
                IMU_NAME,
                ENCODER_RESOLUTION,
                WHEEL_DIAMETER_CM,
                this
        );

        double forward, strafe, rotate, scale; // Drive variables

        // Set motors
        DcMotor slideMotor = (DcMotor)hardwareMap.get(LINEAR_SLIDE_MOTOR_NAME);

        // Set motor behaviors
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double slidePower = 0;
        int slidePosition = 0;

        MINIMUM_SLIDE_POSITION = slideMotor.getCurrentPosition();
        MAXIMUM_SLIDE_POSITION =  MINIMUM_SLIDE_POSITION + SLIDE_LENGTH_ENCODER_TICKS;

        ServoMechanism.Builder leftClawBuilder = new ServoMechanism.Builder();
        leftClawBuilder.setServo(hardwareMap, "leftServo");
        leftClawBuilder.addState("clasped", 0.1); // test and change servoPosition accordingly
        leftClawBuilder.addState("released", 0.4); // test and change servoPosition accordingly
        ServoMechanism leftClaw = leftClawBuilder.build();

        ServoMechanism.Builder rightClawBuilder = new ServoMechanism.Builder();
        rightClawBuilder.setServo(hardwareMap, "rightServo");
        rightClawBuilder.addState("clasped", 0.6); // test and change servoPosition accordingly
        rightClawBuilder.addState("released", 0.4); // test and change servoPosition accordingly
        ServoMechanism rightClaw = rightClawBuilder.build();

        ServoMechanism.Builder rotatorBuilder = new ServoMechanism.Builder();
        rotatorBuilder.setServo(hardwareMap, "rotator");
        rotatorBuilder.addState("collecting", 0.6); // test and change servoPosition accordingly
        rotatorBuilder.addState("scoring", 0.0); // test and change servoPosition accordingly
        ServoMechanism rotator = rotatorBuilder.build();

        ServoMechanism.Builder launcherBuilder = new ServoMechanism.Builder();
        launcherBuilder.setServo(hardwareMap, "launcher");
        launcherBuilder.addState("standby", 1.0); // test and change servoPosition accordingly
        launcherBuilder.addState("launching", 0.0); // test and change servoPosition accordingly
        ServoMechanism launcher = launcherBuilder.build();
        launcher.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            forward = -gamepad1.left_stick_y; // Up is negative; we want up to be positive, so we *(-1)
            strafe = gamepad1.left_stick_x; // Perfect child, no flip
            rotate = -gamepad1.right_stick_x; // Positive is CCW; we want positive to be CW, so we *(-1)
            scale = 0.8;

            mecanum.drive(forward, strafe, rotate, scale);

            // Slide Logic
            if (gamepad2.dpad_up) {
                slidePosition = MAXIMUM_SLIDE_POSITION;
                slidePower = 0.5;
            }
            else if (gamepad2.dpad_down) {
                slidePosition = MINIMUM_SLIDE_POSITION;
                slidePower = -0.2;
            }
            else slidePower = 0;

            // Claw Logic
            if (gamepad2.a) {
                leftClaw.setStateByName("clasped");
                rightClaw.setStateByName("clasped");
            }
            else if (gamepad2.b) {
                leftClaw.setStateByName("released");
                rightClaw.setStateByName("released");
            }
            if (gamepad2.x) {
                rotator.setStateByName("collecting");
            }
            else if (gamepad2.y) {
                rotator.setStateByName("scoring");
            }
//            if (gamepad2.left_bumper && gamepad2.left_bumper && gamepad1.right_bumper && gamepad1.right_bumper) {
            if (gamepad2.left_bumper) {
                launcher.setStateByName("launching");
                telemetry.addData("Launch State", launcher.getCurrentState().stateName);
            } else {
                launcher.setStateByName("standby");
                telemetry.addData("Launch State", launcher.getCurrentState().stateName);
            }


            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setTargetPosition(slidePosition);
            slideMotor.setPower(slidePower);

            telemetry.addData("forward", forward);
            telemetry.addData("strafe", strafe);
            telemetry.addData("rotate", rotate);
            telemetry.addData("slide position", slideMotor.getCurrentPosition());
            telemetry.addData("leftServo", leftClaw.getCurrentState().stateName);
            telemetry.addData("rightServo", rightClaw.getCurrentState().stateName);
            telemetry.addData("rotator Target Position", rotator.getCurrentState().servoPosition);
            telemetry.addData("rotator Position", rotator.getServo().getPosition());
            telemetry.update();
        }

    }

}
