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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "--KURRYTeleOpV.M&K", group = "Linear Opmode")
public class KurryTeleOpFinal extends LinearOpMode {

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

    enum DriveState { DRIVE, ALIGN }
    enum Alliance { RED, BLUE }

    private DriveState state = DriveState.DRIVE;
    private Alliance alliance = Alliance.RED;

    private double allianceSign = -1;

    // ================= PID VARIABLES (ADDED) =================
    private double kP = 0.0008;
    private double kI = 0.0000008;
    private double kD = 0.00015;

    private double leftIntegral = 0, rightIntegral = 0;
    private double leftLastError = 0, rightLastError = 0;

    private double targetVelocityLeft = 0;
    private double targetVelocityRight = 0;

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

            updateLaunchers(); // ===== PID RUNS HERE (ADDED) =====

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

            telemetry.update();
        }
    }
    private void driveControls() {

        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;
        moveRobot(y * SPEED_FACTOR, x * SPEED_FACTOR, rx * SPEED_FACTOR);

        double pL = 0, pR = 0;
        if (gamepad2.left_trigger > 0.2) pL = 0.43;
        else if (gamepad2.left_bumper)  pL = 0.48;
        else if (gamepad2.dpad_up)      pL = 1;

        if (gamepad2.right_trigger > 0.2) pR = 0.467;
        else if (gamepad2.right_bumper) pR = 0.55;
        else if (gamepad2.y) pR =1;

        launcherLeft.setPower(pL);
        launcherRight.setPower(pR);

        if (gamepad1.right_trigger > 0.2) {
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake.setPower(0.8);
        } else if (gamepad1.left_trigger > 0.2) {
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake.setPower(-0.8);
        } else {
            if (intake.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                intake.setTargetPosition(intake.getCurrentPosition());
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setPower(0.5);
            }
        }

        flapperLeft.setPosition(gamepad2.b ? 0.55 : 0.25);
        flapperRight.setPosition(gamepad2.dpad_left ? 0.65 : 0.82);

        if (gamepad2.left_stick_button) servoWheel.setPower(1.0);
        else if (gamepad2.right_stick_button) servoWheel.setPower(-1.0);
        else if (gamepad2.dpad_down) servoWheel.setPower(0);
    }



    private void alignToPost() {

        if (!alignInit) {
            alignTimer.reset();
            lostTagTimer.reset();
            setRunWithoutEncoders();
            lockedTag = null;
            alignInit = true;
        }

        List<AprilTagDetection> detections = aprilTagHelper.getDetections();

        if (detections != null && !detections.isEmpty()) {
            lockedTag = getClosestTag(detections);
            lostTagTimer.reset();
        }

        if (lockedTag == null || lostTagTimer.seconds() > 0.4) {
            moveRobot(0, 0, 0.2 * allianceSign);
            return;
        }

        if (alignTimer.seconds() > 3.0) {
            exitAlign();
            return;
        }

        double rangeError = lockedTag.ftcPose.range ;
        double bearingError = lockedTag.ftcPose.bearing;
        double yawError = lockedTag.ftcPose.yaw;

        if (Math.abs(rangeError) < RANGE_TOL &&
                Math.abs(bearingError) < BEARING_TOL &&
                Math.abs(yawError) < YAW_TOL) {
            gamepad1.rumble(250);
            exitAlign();
            return;
        }

        double drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_DRIVE, MAX_DRIVE);
        double strafe = Range.clip(-bearingError * STRAFE_GAIN, -MAX_STRAFE, MAX_STRAFE);
        double turn   = Range.clip(-yawError * TURN_GAIN, -MAX_TURN, MAX_TURN);

        moveRobot(drive, strafe, turn);
    }

    private AprilTagDetection getClosestTag(List<AprilTagDetection> detections) {
        AprilTagDetection best = null;
        double bestRange = Double.MAX_VALUE;
        for (AprilTagDetection tag : detections) {
            if (tag.ftcPose.range < bestRange) {
                bestRange = tag.ftcPose.range;
                best = tag;
            }
        }
        return best;
    }

    private void exitAlign() {
        stopAll();
        setRunUsingEncoders();
        state = DriveState.DRIVE;
        alignInit = false;
        lockedTag = null;
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

    private void stopAll() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void setRunWithoutEncoders() {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setRunUsingEncoders() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ================= LAUNCHER PID (ADDED AT BOTTOM) =================

    private void updateLaunchers() {

        targetVelocityLeft = 0;
        targetVelocityRight = 0;

        if (gamepad2.left_trigger > 0.2)  targetVelocityLeft  = 1900;
        if (gamepad2.right_trigger > 0.2) targetVelocityRight = 2000;

        if (gamepad2.left_bumper)  targetVelocityLeft  = 1900;
        if (gamepad2.right_bumper) targetVelocityRight = 2000;

        if (gamepad2.dpad_up) targetVelocityLeft  = 4500;
        if (gamepad2.y)       targetVelocityRight = 4500;

        double dt = pidTimer.seconds();
        pidTimer.reset();

        launcherLeft.setVelocityPIDFCoefficients(0.00008,0.00000008,.0005,14);
        launcherRight.setVelocityPIDFCoefficients(0.00008,0.00000008,.0005,14);

        launcherLeft.setVelocity(targetVelocityLeft);
        launcherRight.setVelocity(targetVelocityRight);
    }

    private double launcherPID(double target, double current, double dt, boolean isLeft) {

        double error = target - current;

        if (isLeft) {
            leftIntegral += error * dt;
            double derivative = (error - leftLastError) / dt;
            leftLastError = error;
            return (kP * error) + (kI * leftIntegral) + (kD * derivative);
        } else {
            rightIntegral += error * dt;
            double derivative = (error - rightLastError) / dt;
            rightLastError = error;
            return (kP * error) + (kI * rightIntegral) + (kD * derivative);
        }
    }
}
