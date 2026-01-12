package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "--KURRY TeleOp", group = "Linear Opmode")
public class LedLightsMovement extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor launcherLeft, launcherRight, intake;
    private Servo flapperLeft, flapperRight;
    private CRServo servoWheel;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    private boolean isHoldingTriggerR = false;
    private boolean isHoldingTriggerL = false;
    private int holdPosition = 0;

    // Servo wheel constants
    final double ServoWheelRIGHT = 1;
    final double ServoWheelSTOP = 0;
    final double ServoWheelLEFT = -1;

    // Launcher power presets
    private double leftLauncherPowerMID = 0.57;
    private double rightLauncherPowerMID = 0.6;
    private double leftLauncherXC = 1.0;
    private double rightLauncherXC = 1.0;

    private double flapperLeftPosition = 0.3;
    private double flapperRightPosition = 0.71;

    private static final double SPEED_FACTOR = 0.7;
    private static final double MOVE_THRESHOLD = 0.05;

    @Override
    public void runOpMode() {

        // Hardware mapping
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        intake = hardwareMap.get(DcMotor.class, "intake");

        flapperLeft = hardwareMap.get(Servo.class, "fl");
        flapperRight = hardwareMap.get(Servo.class, "fr");
        servoWheel = hardwareMap.get(CRServo.class, "sw");

        // Motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        launcherLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherRight.setDirection(DcMotor.Direction.FORWARD);

        flapperLeft.setDirection(Servo.Direction.REVERSE);
        flapperRight.setDirection(Servo.Direction.FORWARD);

        // Brake
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // LEDs
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);

        telemetry.addLine("KURRY TeleOp Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Drive
            double leftStickX = gamepad1.left_stick_x * SPEED_FACTOR;
            double leftStickY = -gamepad1.left_stick_y * SPEED_FACTOR;
            double rightStickX = gamepad1.right_stick_x * SPEED_FACTOR;

            double frontLeftPower = leftStickY + leftStickX;
            double frontRightPower = leftStickY - leftStickX;
            double backLeftPower = leftStickY + rightStickX;
            double backRightPower = leftStickY - rightStickX;

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // LED logic — BLUE when moving
            boolean robotMoving =
                    Math.abs(frontLeftPower) > MOVE_THRESHOLD ||
                            Math.abs(frontRightPower) > MOVE_THRESHOLD ||
                            Math.abs(backLeftPower) > MOVE_THRESHOLD ||
                            Math.abs(backRightPower) > MOVE_THRESHOLD;

            if (robotMoving) {
                setNewPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else {
                setNewPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
            }

            // Launcher
            double powerR = 0;
            double powerL = 0;

            if (gamepad2.right_trigger > 0.2) powerR = 0.4;
            if (gamepad2.left_trigger > 0.2) powerL = 0.5;
            if (gamepad2.left_bumper) powerL = leftLauncherPowerMID;
            if (gamepad2.right_bumper) powerR = rightLauncherPowerMID;
            if (gamepad2.dpad_up) powerL = leftLauncherXC;
            if (gamepad2.y) powerR = rightLauncherXC;

            launcherRight.setPower(powerR);
            launcherLeft.setPower(powerL);

            // Intake
            if (gamepad1.right_trigger > 0.2) {
                isHoldingTriggerR = true;
                intake.setPower(0.8);
            } else if (isHoldingTriggerR) {
                isHoldingTriggerR = false;
                holdPosition = intake.getCurrentPosition();
                intake.setTargetPosition(holdPosition);
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setPower(0.3);
            }

            if (gamepad1.left_trigger > 0.2) {
                isHoldingTriggerL = true;
                intake.setPower(-0.8);
            } else if (isHoldingTriggerL) {
                isHoldingTriggerL = false;
                holdPosition = intake.getCurrentPosition();
                intake.setTargetPosition(holdPosition);
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setPower(0.3);
            }

            // Flappers
            flapperLeft.setPosition(gamepad2.b ? 0.14 : flapperLeftPosition);
            flapperRight.setPosition(gamepad2.dpad_left ? 0.58 : flapperRightPosition);

            // Servo wheel
            if (gamepad2.left_stick_button) {
                servoWheel.setPower(ServoWheelRIGHT);
            } else if (gamepad2.right_stick_button) {
                servoWheel.setPower(ServoWheelLEFT);
            } else if (gamepad2.dpad_down) {
                servoWheel.setPower(ServoWheelSTOP);
            }

            telemetry.update();
            sleep(30);
        }
    }

    private void setNewPattern(RevBlinkinLedDriver.BlinkinPattern newPattern) {
        if (pattern != newPattern) {
            pattern = newPattern;
            blinkinLedDriver.setPattern(pattern);
        }
    }
}
