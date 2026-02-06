package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "---KURRYTeleOpV FINAL", group = "Linear Opmode")
public class KurryTeleOpFinalThursdayBackup extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight, intake;
    private DcMotorEx launcherLeft, launcherRight;
    private Servo flapperLeft, flapperRight;
    private CRServo servoWheel;
    private IMU imu;

    private AprilTagHelper aprilTagHelper;
    private AprilTagDetection lockedTag = null;

    private ElapsedTime alignTimer = new ElapsedTime();
    private ElapsedTime lostTagTimer = new ElapsedTime();

    private boolean alignInit = false;

    private static final double SPEED_FACTOR = 0.7;

    private static final double SPEED_GAIN  = 0.03;
    private static final double STRAFE_GAIN = 0.02;
    private static final double TURN_GAIN   = 0.01;

    private static final double MAX_DRIVE  = 0.5;
    private static final double MAX_STRAFE = 0.5;
    private static final double MAX_TURN   = 0.3;

    private static final double TARGET_RANGE = 60.0;
    private static final double RANGE_TOL = 1.0;
    private static final double BEARING_TOL = 1.0;
    private static final double YAW_TOL = 1.0;

    final double ServoWheelRIGHT = 1;
    final double ServoWheelSTOP = 0;
    final double ServoWheelLEFT = -1;

    enum DriveState { DRIVE, ALIGN }
    enum Alliance { RED, BLUE }

    private DriveState state = DriveState.DRIVE;
    private Alliance alliance = Alliance.RED;

    private double allianceSign = -1;

    // ================= PID CONSTANTS =================
    private double kP = 0.0008;
    private double kI = 0.0000008;
    private double kD = 0.00015;

    private double leftIntegral = 0, rightIntegral = 0;
    private double leftLastError = 0, rightLastError = 0;

    private double targetVelocityLeft = 0;
    private double targetVelocityRight = 0;

    // ===== PID TELEMETRY VARIABLES =====
    private double leftError, rightError;
    private double leftOutput, rightOutput;

    private ElapsedTime pidTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        frontLeft   = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight  = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft    = hardwareMap.get(DcMotor.class, "backLeft");
        backRight   = hardwareMap.get(DcMotor.class, "backRight");
        launcherLeft  = hardwareMap.get(DcMotorEx.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flapperLeft = hardwareMap.get(Servo.class, "fl");
        flapperRight = hardwareMap.get(Servo.class, "fr");
        servoWheel = hardwareMap.get(CRServo.class, "sw");
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);

        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));

        aprilTagHelper = new AprilTagHelper(hardwareMap);

        while (!isStarted()) {
            if (gamepad1.b) alliance = Alliance.RED;
            if (gamepad1.x) alliance = Alliance.BLUE;
            allianceSign = (alliance == Alliance.BLUE) ? 1 : -1;
            telemetry.addData("Alliance", alliance);
            telemetry.update();
        }

        waitForStart();
        pidTimer.reset();

        while (opModeIsActive()) {

            updateLaunchers();

            if (state == DriveState.DRIVE) {
                driveControls();
                if (gamepad1.a) {
                    state = DriveState.ALIGN;
                    alignInit = false;
                }
            } else {
                alignToPost();
                if (gamepad1.b) exitAlign();
            }

            // ========== LAUNCHER PID TELEMETRY ==========
            telemetry.addLine("==== LAUNCHER PID ====");
            telemetry.addData("Left Target", "%.0f", targetVelocityLeft);
            telemetry.addData("Left Velocity", "%.0f", launcherLeft.getVelocity());
            telemetry.addData("Left Error", "%.0f", leftError);
            telemetry.addData("Left Power", "%.3f", leftOutput);
            telemetry.addData("Left Integral", "%.2f", leftIntegral);

            telemetry.addLine();

            telemetry.addData("Right Target", "%.0f", targetVelocityRight);
            telemetry.addData("Right Velocity", "%.0f", launcherRight.getVelocity());
            telemetry.addData("Right Error", "%.0f", rightError);
            telemetry.addData("Right Power", "%.3f", rightOutput);
            telemetry.addData("Right Integral", "%.2f", rightIntegral);

            telemetry.update();
        }
    }

    private void driveControls() {

        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;
        moveRobot(y * SPEED_FACTOR, x * SPEED_FACTOR, rx * SPEED_FACTOR);

        if (gamepad1.right_trigger > 0.2) intake.setPower(1);
        else if (gamepad1.left_trigger > 0.2) intake.setPower(-1);
        else intake.setPower(0);

        flapperLeft.setPosition(gamepad2.b ? 0.65 : 0.05);
        flapperRight.setPosition(gamepad2.dpad_left ? 0.65 : 0.82);

        if (gamepad2.left_stick_button) {
            servoWheel.setPower(ServoWheelRIGHT);
        } else if (gamepad2.right_stick_button) {
            servoWheel.setPower(ServoWheelLEFT);
        } else if (gamepad2.dpad_down) {
            servoWheel.setPower(ServoWheelSTOP);
        }


    }


    private void updateLaunchers() {

        targetVelocityLeft = 0;
        targetVelocityRight = 0;

        if (gamepad2.left_trigger > 0.2)  targetVelocityLeft  = 1300;
        if (gamepad2.right_trigger > 0.2) targetVelocityRight = 1300;

        if (gamepad2.left_bumper)  targetVelocityLeft  = 1350;
        if (gamepad2.right_bumper) targetVelocityRight = 1800;

        if (gamepad2.dpad_up) targetVelocityLeft  = 4500;
        if (gamepad2.y)       targetVelocityRight = 4500;

        launcherLeft.setVelocityPIDFCoefficients(0.0009,0.0000006,0.00005,14);
        launcherRight.setVelocityPIDFCoefficients(0.0009,0.0000006,0.00005,14);
        launcherLeft.setVelocity(targetVelocityLeft);
        launcherRight.setVelocity(targetVelocityRight);

    }




    private void moveRobot(double y, double x, double rx) {
        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;
        double max = Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1) { fl /= max; fr /= max; bl /= max; br /= max; }
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void alignToPost() {}
    private void exitAlign() {}
}
