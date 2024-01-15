package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Helper.MecanumDrive;
import org.firstinspires.ftc.teamcode.Helper.ServoMechanism;

@TeleOp(name="Fighting Cougars Main TeleOp", group="Tests")
public class FCMainTeleOp extends LinearOpMode {

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

    // Drive state variables
    public enum DriveState {NORMAL, SLOW};
    DriveState driveState = DriveState.NORMAL;
    boolean currentDrivePress = false;
    boolean previousDrivePress = false;

    // Rotator button variables
    boolean currentRotatorPress = false;
    boolean previousRotatorPress = false;

    // Claw button variables
    boolean currentClawPress = false;
    boolean previousClawPress = false;

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
        scale = 1.0;
        // Slide variables
        double slidePower = 0;
        int slidePosition = 0;

        // Set motors
        DcMotor slideMotor = (DcMotor)hardwareMap.get(LINEAR_SLIDE_MOTOR_NAME);

        // Set motor behaviors
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set slide positions relative to the initial position
        MINIMUM_SLIDE_POSITION = slideMotor.getCurrentPosition();
        MAXIMUM_SLIDE_POSITION =  MINIMUM_SLIDE_POSITION + SLIDE_LENGTH_ENCODER_TICKS;

        // Create left claw servo
        ServoMechanism.Builder leftClawBuilder = new ServoMechanism.Builder();
        leftClawBuilder.setServo(hardwareMap, "leftServo");
        leftClawBuilder.addState("clasped", 0.1); // test and change servoPosition accordingly
        leftClawBuilder.addState("released", 0.4); // test and change servoPosition accordingly
        ServoMechanism leftClaw = leftClawBuilder.build();

        // Create right claw servo
        ServoMechanism.Builder rightClawBuilder = new ServoMechanism.Builder();
        rightClawBuilder.setServo(hardwareMap, "rightServo");
        rightClawBuilder.addState("clasped", 0.6); // test and change servoPosition accordingly
        rightClawBuilder.addState("released", 0.0); // test and change servoPosition accordingly
        ServoMechanism rightClaw = rightClawBuilder.build();

        // Create rotator servo
        ServoMechanism.Builder rotatorBuilder = new ServoMechanism.Builder();
        rotatorBuilder.setServo(hardwareMap, "rotator");
        rotatorBuilder.addState("scoring", 0.6); // test and change servoPosition accordingly
        rotatorBuilder.addState("collecting", 0.1); // test and change servoPosition accordingly
        ServoMechanism rotator = rotatorBuilder.build();

        // Create launcher servo
        ServoMechanism.Builder launcherBuilder = new ServoMechanism.Builder();
        launcherBuilder.setServo(hardwareMap, "launcher");
        launcherBuilder.addState("standby", 0.9); // test and change servoPosition accordingly
        launcherBuilder.addState("launching", 0.0); // test and change servoPosition accordingly
        ServoMechanism launcher = launcherBuilder.build();
        // launcher.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            // Gather inputs from driver controller
            forward = -gamepad1.left_stick_y; // Up is negative; we want up to be positive, so we *(-1)
            strafe = gamepad1.left_stick_x; // Perfect child, no flip
            rotate = -gamepad1.right_stick_x; // Positive is CCW; we want positive to be CW, so we *(-1)

            currentDrivePress = gamepad1.a;

//            if (currentPress == true && previousPress == false) {
//                RobotLog.v("Toggled!");
//                previousPress = currentPress;
//                if (driveState == DriveState.NORMAL) { driveState = DriveState.SLOW; }
//                else if (driveState == DriveState.SLOW) { driveState = DriveState.NORMAL; }
//            } else if (currentPress == false && previousPress == true) {
//                RobotLog.v("Released!");
//                previousPress = currentPress;
//            }

            if (gamepad1.right_trigger > 0.8f) { driveState = DriveState.SLOW; }
            else { driveState = DriveState.NORMAL; }

            if (driveState == DriveState.NORMAL) { scale = 1.0; }
            else if (driveState == DriveState.SLOW) { scale = 0.6; }

            // Update mecanum drive train
            mecanum.drive(forward, strafe, rotate, scale);

            // Slide Logic
            if (gamepad2.dpad_up) {
                slidePosition = MAXIMUM_SLIDE_POSITION;
                slidePower = 0.7;
            } else if (gamepad2.dpad_down) {
                slidePosition = MINIMUM_SLIDE_POSITION;
                slidePower = -0.3;
            } else slidePower = 0;

            slideMotor.setTargetPosition(slidePosition);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(slidePower);

            // Claw Logic
            currentClawPress = gamepad2.a;
            if (currentClawPress == true && previousClawPress == false) {
                previousClawPress = currentClawPress;
                if (leftClaw.getCurrentState().stateName == "released" && rightClaw.getCurrentState().stateName == "released") {
                    leftClaw.setStateByName("clasped");
                    rightClaw.setStateByName("clasped");
                }
                else if (leftClaw.getCurrentState().stateName == "clasped" && rightClaw.getCurrentState().stateName == "clasped") {
                    leftClaw.setStateByName("released");
                    rightClaw.setStateByName("released");
                }
            }
            else if (currentClawPress == false && previousClawPress == true) { previousClawPress = currentClawPress; }

            // Rotator Logic
            currentRotatorPress = gamepad2.y;
            if (currentRotatorPress == true && previousRotatorPress == false) {
                previousRotatorPress = currentRotatorPress;
                if (rotator.getCurrentState().stateName == "collecting") { rotator.setStateByName("scoring"); }
                else if (rotator.getCurrentState().stateName == "scoring") { rotator.setStateByName("collecting"); }
            }
            else if (currentRotatorPress == false && previousRotatorPress == true) { previousRotatorPress = currentRotatorPress; }

            // Launcher Logic
            if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad2.left_bumper && gamepad2.right_bumper)
//            if (gamepad2.left_bumper)
            { launcher.setStateByName("launching"); }
            else { launcher.setStateByName("standby"); }

            // Telemetry
            String driveTelemetry = String.valueOf(forward) + ", " + String.valueOf(strafe) + ", " + String.valueOf(rotate);
            telemetry.addData("forward, strafe, rotate", driveTelemetry);
            telemetry.addData("slide position", slideMotor.getCurrentPosition());
            telemetry.addData("leftClaw State, rightClaw State", leftClaw.getCurrentState().stateName + ", " + rightClaw.getCurrentState().stateName);
            telemetry.addData("rotator State, rotator Position", rotator.getCurrentState().stateName + ", " + rotator.getServo().getPosition());
            telemetry.addData("launcher State, launcher Position", launcher.getCurrentState().stateName + ", " + launcher.getServo().getPosition());
            telemetry.addData("driving mode", driveState);
            telemetry.update();
        }

    }

}
