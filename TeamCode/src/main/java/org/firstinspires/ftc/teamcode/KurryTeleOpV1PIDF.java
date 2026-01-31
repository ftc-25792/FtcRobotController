package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "--KURRY TeleOp.V67 PIDF", group = "Linear Opmode")
public class KurryTeleOpV1PIDF extends LinearOpMode {

    // ================= DRIVE MOTORS =================
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // ================= MECHANISMS =================
    private DcMotorEx launcherLeft, launcherRight;
    private DcMotor intake;

    private Servo flapperLeft, flapperRight;
    private CRServo servoWheel;

    // ================= CONSTANTS =================
    private static final double SPEED_FACTOR = 0.7;

    final double ServoWheelRIGHT = 1;
    final double ServoWheelSTOP  = 0;
    final double ServoWheelLEFT  = -1;

    private double flapperLeftPosition = 0.3;
    private double flapperRightPosition = 0.71;

    // ================= PIDF CONSTANTS =================
    // PIDF tuned for RS-555 6000 RPM
    private double kP = 0.0006;
    private double kI = 0.0000005;
    private double kD = 0.0001;
    private double kF = 0.00036;

    // ================= PIDF STATE =================
    private double leftIntegral = 0, rightIntegral = 0;
    private double leftLastError = 0, rightLastError = 0;

    private double targetVelocityLeft = 0;
    private double targetVelocityRight = 0;

    private ElapsedTime pidTimer = new ElapsedTime();

    // ================= INTAKE HOLD =================
    private boolean isHoldingTriggerR = false;
    private boolean isHoldingTriggerL = false;
    private int holdPosition = 0;

    @Override
    public void runOpMode() {

        // ================= HARDWARE MAP =================
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        launcherLeft  = hardwareMap.get(DcMotorEx.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");

        intake = hardwareMap.get(DcMotor.class, "intake");

        flapperLeft  = hardwareMap.get(Servo.class, "fl");
        flapperRight = hardwareMap.get(Servo.class, "fr");
        servoWheel   = hardwareMap.get(CRServo.class, "sw");

        // ================= DIRECTIONS =================
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        launcherLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherRight.setDirection(DcMotor.Direction.FORWARD);

        flapperLeft.setDirection(Servo.Direction.REVERSE);
        flapperRight.setDirection(Servo.Direction.FORWARD);

        // ================= MODES =================
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("KURRY TeleOp PIDF Initialized");
        telemetry.update();

        waitForStart();
        pidTimer.reset();

        // ================= MAIN LOOP =================
        while (opModeIsActive()) {

            // ================= DRIVE =================
            double lx = gamepad1.left_stick_x * SPEED_FACTOR;
            double ly = -gamepad1.left_stick_y * SPEED_FACTOR;
            double rx = gamepad1.right_stick_x * SPEED_FACTOR;

            frontLeft.setPower(ly + lx);
            frontRight.setPower(ly - lx);
            backLeft.setPower(ly + rx);
            backRight.setPower(ly - rx);

            // ================= LAUNCHER TARGETS =================
            targetVelocityLeft = 0;
            targetVelocityRight = 0;

            if (gamepad2.left_trigger > 0.2)  targetVelocityLeft  = 1600;
            if (gamepad2.right_trigger > 0.2) targetVelocityRight = 1700;

            if (gamepad2.left_bumper)  targetVelocityLeft  = 1900;
            if (gamepad2.right_bumper) targetVelocityRight = 2000;

            if (gamepad2.dpad_up) targetVelocityLeft = 2600;
            if (gamepad2.y)       targetVelocityRight = 2600;

            // ================= PIDF CALC =================
            double dt = pidTimer.seconds();
            pidTimer.reset();

            // LEFT
            double leftVel = launcherLeft.getVelocity();
            double leftError = targetVelocityLeft - leftVel;
            leftIntegral += leftError * dt;
            double leftDerivative = (leftError - leftLastError) / dt;

            double leftPower =
                    (kF * targetVelocityLeft) +
                            (kP * leftError) +
                            (kI * leftIntegral) +
                            (kD * leftDerivative);

            leftLastError = leftError;

            // RIGHT
            double rightVel = launcherRight.getVelocity();
            double rightError = targetVelocityRight - rightVel;
            rightIntegral += rightError * dt;
            double rightDerivative = (rightError - rightLastError) / dt;

            double rightPower =
                    (kF * targetVelocityRight) +
                            (kP * rightError) +
                            (kI * rightIntegral) +
                            (kD * rightDerivative);

            rightLastError = rightError;

            launcherLeft.setPower(Math.max(-1, Math.min(1, leftPower)));
            launcherRight.setPower(Math.max(-1, Math.min(1, rightPower)));

            // ================= INTAKE =================
            if (gamepad1.right_trigger > 0.2) {
                isHoldingTriggerR = true;
                intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
                intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intake.setPower(-0.8);
            } else if (isHoldingTriggerL) {
                isHoldingTriggerL = false;
                holdPosition = intake.getCurrentPosition();
                intake.setTargetPosition(holdPosition);
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setPower(0.8);
            }

            // ================= FLAPPERS =================
            flapperLeft.setPosition(gamepad2.b ? 0.14 : flapperLeftPosition);
            flapperRight.setPosition(gamepad2.dpad_left ? 0.58 : flapperRightPosition);

            // ================= SERVO WHEEL =================
            if (gamepad2.left_stick_button) {
                servoWheel.setPower(ServoWheelRIGHT);
            } else if (gamepad2.right_stick_button) {
                servoWheel.setPower(ServoWheelLEFT);
            } else if (gamepad2.dpad_down) {
                servoWheel.setPower(ServoWheelSTOP);
            }

            // ================= TELEMETRY =================
            telemetry.addData("Target L", targetVelocityLeft);
            telemetry.addData("Target R", targetVelocityRight);
            telemetry.addData("Vel L", "%.0f", launcherLeft.getVelocity());
            telemetry.addData("Vel R", "%.0f", launcherRight.getVelocity());
            telemetry.update();

            sleep(50);
        }
    }
}
