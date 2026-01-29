package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
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

    private boolean isHoldingTriggerR = false;
    private boolean isHoldingPositionR = false;
    private boolean isHoldingTriggerL = false;
    private boolean isHoldingPositionL = false;
    private int holdPosition = 0;

    private KurryStateDrive CurrentState = KurryStateDrive.eDrive;

    // Servo constants
    final double ServoWheelRIGHT = 1;
    final double ServoWheelSTOP = 0;
    final double ServoWheelLEFT = -1;

    // Launcher speed presets
    private double leftLauncherPowerMID = 0.48;
    private double rightLauncherPowerMID = 0.55;
    private double leftLauncherXC = 0.75;
    private double rightLauncherXC = 0.75;

    private static final double SPEED_FACTOR = 0.7;

    // Align POST constants
    private static final double SPEED_GAIN = 0.03;
    private static final double TURN_GAIN = 0.01;
    private static final double STRAFE_GAIN = 0.02;
    private static final double MAX_AUTO_SPEED = 0.5;
    private static final double MAX_AUTO_TURN = 0.3;
    private static final double MAX_AUTO_STRAFE = 0.5;

    private AprilTagHelper aprilTagHelper;
    private AprilTagDetection PostTag = null;

    enum KurryStateDrive {
        eDrive,
        eAlign_POST
    }

    @Override
    public void runOpMode() {
        // Hardware mapping
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");
        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flapperLeft = hardwareMap.get(Servo.class, "fl");
        flapperRight = hardwareMap.get(Servo.class, "fr");
        servoWheel = hardwareMap.get(CRServo.class, "sw");

        // Motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherRight.setDirection(DcMotor.Direction.FORWARD);

        // Servo directions
        flapperLeft.setDirection(Servo.Direction.REVERSE);
        flapperRight.setDirection(Servo.Direction.FORWARD);

        // Zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();

        aprilTagHelper = new AprilTagHelper(hardwareMap);

        telemetry.addLine("KURRY TeleOp Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            switch (CurrentState) {
                case eDrive:
                    driveControls();
                    if (gamepad1.a) {
                        CurrentState = KurryStateDrive.eAlign_POST;
                    }
                    break;

                case eAlign_POST:
                    alignPostTeleOp();
                    if (gamepad1.b) {
                        stopAll();
                        CurrentState = KurryStateDrive.eDrive;
                    }
                    break;
            }
            telemetry.update();
        }
    }

    private void driveControls() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        y *= SPEED_FACTOR;
        x *= SPEED_FACTOR;
        rx *= SPEED_FACTOR;

        moveRobot(y, x, rx);

        // Launcher control
        double powerR = 0;
        double powerL = 0;

        if (gamepad2.right_trigger > 0.2) powerR = 0.467;
        if (gamepad2.left_trigger > 0.2) powerL = 0.43;
        if (gamepad2.left_bumper) powerL = leftLauncherPowerMID;
        if (gamepad2.right_bumper) powerR = rightLauncherPowerMID;
        if (gamepad2.dpad_up) powerL = leftLauncherXC;
        if (gamepad2.y) powerR = rightLauncherXC;

        launcherRight.setPower(powerR);
        launcherLeft.setPower(powerL);

        // Intake control
        if (gamepad1.right_trigger > 0.2) {
            isHoldingTriggerR = true;
            isHoldingPositionR = false;
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake.setPower(0.8);
        } else if (isHoldingTriggerR) {
            isHoldingTriggerR = false;
            isHoldingPositionR = true;
            holdPosition = intake.getCurrentPosition();
            intake.setTargetPosition(holdPosition);
            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.setPower(0.3);
        }

        if (gamepad1.left_trigger > 0.2) {
            isHoldingTriggerL = true;
            isHoldingPositionL = false;
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake.setPower(-0.8);
        } else if (isHoldingTriggerL) {
            isHoldingTriggerL = false;
            isHoldingPositionL = true;
            holdPosition = intake.getCurrentPosition();
            intake.setTargetPosition(holdPosition);
            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.setPower(0.8);
        }

        // Flapper control
        if (gamepad2.b) flapperLeft.setPosition(0.4);
        else flapperLeft.setPosition(0.55);

        if (gamepad2.dpad_left) flapperRight.setPosition(0.65);
        else flapperRight.setPosition(0.82);

        // Servo wheel control
        if (gamepad2.left_stick_button) servoWheel.setPower(ServoWheelRIGHT);
        else if (gamepad2.right_stick_button) servoWheel.setPower(ServoWheelLEFT);
        else if (gamepad2.dpad_down) servoWheel.setPower(ServoWheelSTOP);
    }

    private void alignPostTeleOp() {
        List<AprilTagDetection> detections = aprilTagHelper.getDetections();

        if (detections != null && !detections.isEmpty()) {
            PostTag = detections.get(0);

            double rangeError = PostTag.ftcPose.range - 40;
            double bearingError = PostTag.ftcPose.bearing;
            double yawError = PostTag.ftcPose.yaw;

            double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn = Range.clip(bearingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            moveRobot(drive, strafe, turn);

            telemetry.addData("Range", "%5.1f", PostTag.ftcPose.range);
            telemetry.addData("Bearing", "%5.1f", PostTag.ftcPose.bearing);
            telemetry.addData("Yaw", "%5.1f", PostTag.ftcPose.yaw);
        } else {
            telemetry.addLine("No POST detected!");
            stopAll();
        }
    }

    private void moveRobot(double y, double x, double rx) {
        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));

        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

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
}
