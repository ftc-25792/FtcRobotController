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
    private ElapsedTime stateTimer = new ElapsedTime();
    private boolean targetHeadingInit = false;
    private KurryStateDrive CurrentState = KurryStateDrive.eDrive;
    private double diff = -8;

    private static final double SPEED_FACTOR = 0.7;
    private Alliance alliance = Alliance.eRed;
    private static double SIGN_Alliance = -1;

    private static final double SPEED_GAIN = 0.03;
    private static final double TURN_GAIN  = 0.01;
    private static final double STRAFE_GAIN = 0.02;
    private static final double MAX_AUTO_SPEED = 0.5;
    private static final double MAX_AUTO_TURN  = 0.3;
    private static final double MAX_AUTO_STRAFE = 0.5;

    private static final double RANGE_TOLERANCE = 1;
    private static final double BEARING_TOLERANCE = 1.0;
    private static final double YAW_TOLERANCE = 1.0;

    private AprilTagHelper aprilTagHelper;
    private AprilTagDetection firstTag = null;

    enum KurryStateDrive { eDrive, eAlign_POST }
    enum Alliance { eRed, eBlue }

    @Override
    public void runOpMode() {

        frontLeft   = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight  = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft    = hardwareMap.get(DcMotor.class, "backLeft");
        backRight   = hardwareMap.get(DcMotor.class, "backRight");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");
        launcherLeft  = hardwareMap.get(DcMotorEx.class, "launcherLeft");
        intake      = hardwareMap.get(DcMotor.class, "intake");
        flapperLeft = hardwareMap.get(Servo.class, "fl");
        flapperRight = hardwareMap.get(Servo.class, "fr");
        servoWheel  = hardwareMap.get(CRServo.class, "sw");
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        launcherLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherRight.setDirection(DcMotor.Direction.FORWARD);
        flapperLeft.setDirection(Servo.Direction.REVERSE);
        flapperRight.setDirection(Servo.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));

        aprilTagHelper = new AprilTagHelper(hardwareMap);

        telemetry.addLine("Ready to Start");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.b) alliance = Alliance.eRed;
            if (gamepad1.x) alliance = Alliance.eBlue;

            SIGN_Alliance = (alliance == Alliance.eBlue) ? 1 : -1;

            telemetry.addData("Alliance", alliance);
            telemetry.addData("Sign", SIGN_Alliance);
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            switch (CurrentState) {
                case eDrive:
                    driveControls();
                    if (gamepad1.a) CurrentState = KurryStateDrive.eAlign_POST;
                    break;
                case eAlign_POST:
                    alignPostTeleOp();
                    if (gamepad1.b) { stopAll(); CurrentState = KurryStateDrive.eDrive; firstTag = null; }
                    break;
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
        else if (gamepad2.dpad_up)      pL = 0.75;

        if (gamepad2.right_trigger > 0.2) pR = 0.467;
        else if (gamepad2.right_bumper) pR = 0.55;
        else if (gamepad2.y) pR = 0.75;

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

        flapperLeft.setPosition(gamepad2.b ? 0.4 : 0.55);
        flapperRight.setPosition(gamepad2.dpad_left ? 0.65 : 0.82);

        if (gamepad2.left_stick_button) servoWheel.setPower(1.0);
        else if (gamepad2.right_stick_button) servoWheel.setPower(-1.0);
        else if (gamepad2.dpad_down) servoWheel.setPower(0);
    }

    private void alignPostTeleOp() {
        if(!targetHeadingInit) {
            // Reset timer so we can timeout if tag doesnt align in time
            stateTimer.reset();
            setMotorsNOTUsingEncoders();
            targetHeadingInit = true;
        }

        // Get latest tag detection continuously
        List<AprilTagDetection> detections = aprilTagHelper.getDetections();
        if (detections != null && !detections.isEmpty()) {
            firstTag = detections.get(0); // always use the latest
        }

        if (firstTag == null) {
            telemetry.addLine("No POST detected - Searching...");
            stopAll();
            return;
        }

        // Calculate errors
        double rangeError = firstTag.ftcPose.range - 60;
        double bearingError = firstTag.ftcPose.bearing - (4*SIGN_Alliance);
        double yawError = firstTag.ftcPose.yaw;

        // Check if aligned
        boolean isDistanceAligned = Math.abs(rangeError) < RANGE_TOLERANCE;
        boolean isBearingAligned  = Math.abs(bearingError) < BEARING_TOLERANCE;
        boolean isYawAligned      = Math.abs(yawError) < YAW_TOLERANCE;

        if (isDistanceAligned && isBearingAligned && isYawAligned) {
            stopAll();
            gamepad1.rumble(250);
            CurrentState = KurryStateDrive.eDrive;
            firstTag = null;
            return;
        }
        if(stateTimer.milliseconds() >= 1000)
        {
            CurrentState = KurryStateDrive.eDrive;
            return;
        }

        // Apply gains with clamping to prevent spinning
        double drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double strafe = Range.clip(-bearingError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        double turn   = Range.clip(-yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        if (Math.abs(drive)  < 0.02) drive = 0;
        if (Math.abs(strafe) < 0.02) strafe = 0;
        if (Math.abs(turn)   < 0.02) turn = 0;

        // Move the robot
        moveRobot(drive, strafe, turn);

        // Telemetry for debugging
        telemetry.addLine("AutoAlign: Active");
        telemetry.addData("Errors (R/B/Y)", "%5.2f / %5.2f / %5.2f", rangeError, bearingError, yawError);
        telemetry.addData("Outputs (D/S/T)", "%5.2f / %5.2f / %5.2f", drive, strafe, turn);
    }

    private void setMotorsNOTUsingEncoders(){
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void moveRobot(double y, double x, double rx) {
        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }

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
