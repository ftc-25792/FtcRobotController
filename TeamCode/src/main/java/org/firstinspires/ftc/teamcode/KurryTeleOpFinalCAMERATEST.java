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

@TeleOp(name = "--KURRYTeleOpV.CAMERATEST", group = "Linear Opmode")
public class KurryTeleOpFinalCAMERATEST extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight, intake;
    private DcMotorEx launcherLeft, launcherRight;
    private Servo flapperLeft, flapperRight;
    private CRServo servoWheel;
    private IMU imu;

    private AprilTagHelper aprilTagHelper;

    private static final double SPEED_FACTOR = 0.7;

    private static final double TURN_GAIN = 0.01;
    private static final double MAX_TURN = 0.3;

    // ===== APRILTAG ALIGN CONSTANTS =====
    private static final double SEARCH_TURN = 0.25;
    private static final double ALIGN_TURN_GAIN = 0.01;
    private static final double YAW_TOLERANCE = 1.5;

    final double ServoWheelRIGHT = 1;
    final double ServoWheelSTOP = 0;
    final double ServoWheelLEFT = -1;

    enum DriveState { DRIVE, ALIGN }
    enum Alliance { RED, BLUE }

    private DriveState state = DriveState.DRIVE;
    private Alliance alliance = Alliance.RED;
    private double allianceSign = -1;

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

        while (opModeIsActive()) {

            updateLaunchers();

            if (state == DriveState.DRIVE) {
                driveControls();

                if (gamepad1.a) {
                    state = DriveState.ALIGN;
                }

            } else {
                alignToPost();

                if (gamepad1.b) {
                    exitAlign();
                }
            }

            telemetry.update();
        }
    }

    private void driveControls() {

        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        moveRobot(y * SPEED_FACTOR, x * SPEED_FACTOR, rx * SPEED_FACTOR);

        if (gamepad1.right_trigger > 0.2) intake.setPower(0.8);
        else if (gamepad1.left_trigger > 0.2) intake.setPower(-0.8);
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

        double leftTarget = 0;
        double rightTarget = 0;

        if (gamepad2.left_trigger > 0.2)  leftTarget  = 1300;
        if (gamepad2.right_trigger > 0.2) rightTarget = 1300;

        if (gamepad2.left_bumper)  leftTarget  = 1450;
        if (gamepad2.right_bumper) rightTarget = 1550;

        launcherLeft.setVelocityPIDFCoefficients(0.0008,0.0000008,0.00015,14);
        launcherRight.setVelocityPIDFCoefficients(0.0008,0.0000008,0.00015,14);

        launcherLeft.setVelocity(leftTarget);
        launcherRight.setVelocity(rightTarget);
    }

    private void moveRobot(double y, double x, double rx) {

        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));

        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    // ================= APRILTAG ALIGN =================
    private void alignToPost() {

        List<AprilTagDetection> detections = aprilTagHelper.getDetections();

        // No tag → spin
        if (detections.isEmpty()) {
            moveRobot(0, 0, SEARCH_TURN * allianceSign);
            telemetry.addLine("Searching for AprilTag...");
            return;
        }

        AprilTagDetection tag = detections.get(0);
        double yaw = tag.ftcPose.yaw;

        // Aligned → stop
        if (Math.abs(yaw) <= YAW_TOLERANCE) {
            moveRobot(0, 0, 0);
            telemetry.addLine("Aligned with AprilTag");
            return;
        }

        double turn = Range.clip(
                yaw * ALIGN_TURN_GAIN,
                -MAX_TURN,
                MAX_TURN
        );

        moveRobot(0, 0, turn);

        telemetry.addData("Tag Yaw", yaw);
        telemetry.addData("Turn", turn);
    }

    private void exitAlign() {
        moveRobot(0, 0, 0);
        state = DriveState.DRIVE;
    }
}
