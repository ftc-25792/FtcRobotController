package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.util.List;

@Autonomous(name = "--KurryAutoShootTT", group = "Linear Opmode")
public class KurryAutoShortScore extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor launcherLeft, launcherRight, intake;
    private IMU imu;
    private Servo flapperRight, flapperLeft;
    private CRServo divider;

    private ElapsedTime runtime = new ElapsedTime();
    private AprilTagHelper aprilTagHelper;
    private String pattern = "UNKNOWN";

    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.5;
    private static final double HEADING_THRESHOLD = 1.0;
    private static final double TICKS_PER_INCH = 537.7 / (Math.PI * 4); // 4" wheels, 537.7 ticks/rev

    // Telemetry variables
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private double headingError = 0;

    @Override
    public void runOpMode() {

        // ===== Initialize Motors & Servos =====
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        launcherLeft = hardwareMap.get(DcMotor.class, "launcherRight"); // swapped intentionally
        launcherRight = hardwareMap.get(DcMotor.class, "launcherLeft");
        launcherLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherRight.setDirection(DcMotorSimple.Direction.FORWARD);

        intake = hardwareMap.get(DcMotor.class, "intake");

        flapperRight = hardwareMap.get(Servo.class, "fr");
        flapperLeft = hardwareMap.get(Servo.class, "fl");
        flapperRight.setDirection(Servo.Direction.FORWARD);
        flapperLeft.setDirection(Servo.Direction.REVERSE);

        divider = hardwareMap.get(CRServo.class, "sw");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();

        waitForStart();
        aprilTagHelper = new AprilTagHelper(hardwareMap);
        telemetry.addLine("AprilTag Helper initialized...");
        telemetry.update();
        sleep(500);



        List detections = aprilTagHelper.getDetections();
        while (detections.isEmpty()) {
            aprilTagHelper.telemetryAprilTag(telemetry);
            telemetry.update();
            detections = aprilTagHelper.getDetections();
           // sleep(1000);

        }
        telemetry.addLine("outside while");
        telemetry.update();

        {
            int id = ((org.firstinspires.ftc.vision.apriltag.AprilTagDetection) detections.get(0)).id;
            switch (id) {
                case 21: pattern = "GPP"; break;
                case 22: pattern = "PGP"; break;
                case 23: pattern = "PPG"; break;
            }
            telemetry.addData("Pattern detected", pattern);

            telemetry.update();
            sleep(1000);
        }


       // waitForStart();

        launcherLeft.setPower(0.5);
        launcherRight.setPower(0.5);
        sleep(1000);

//        driveStraight(2, true);        // move into shooting position
        launch(pattern);               // shoot according to pattern
//        driveStraight(8, true);        // continue autonomous

        // Stop camera
        aprilTagHelper.stop();
    }

    private void launch(String pattern) {
        divider.setPower(1);
        intake.setPower(0.8);
        sleep(500);

        switch (pattern) {
            case "PPG":
                rightLaunch();
                divide();
                rightLaunch();
                leftLaunch();

                break;
            case "PGP":
//                sleep(1000);
                rightLaunch();
                leftLaunch();
                divide();
                rightLaunch();
                break;
            case "GPP":
//                sleep(1000);
                leftLaunch();
                rightLaunch();
                divide();
                rightLaunch();
                break;
            default: // fallback
                leftLaunch();
                rightLaunch();
                break;
        }

        divider.setPower(0);
        intake.setPower(0);
        launcherLeft.setPower(0);
        launcherRight.setPower(0);
    }
    private void divide(){
        sleep(500);
        divider.setPower(1);//move to the right
        sleep(1000);
    }

    private void rightLaunch() {
        flapperRight.setPosition(0.58);
        sleep(1000);
        flapperRight.setPosition(0.71);
        sleep(1000);
    }

    private void leftLaunch() {
        flapperLeft.setPosition(0.14);
        sleep(1000);
        flapperLeft.setPosition(0.3);
        sleep(1000);
    }

    private void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setAllPower(double p) {
        frontLeft.setPower(p);
        frontRight.setPower(p);
        backLeft.setPower(p);
        backRight.setPower(p);
    }

    private void stopAll() {
        setAllPower(0);
    }

    private void driveStraight(double inches, boolean forward) {
        int move = (int) (inches * TICKS_PER_INCH);
        if (!forward) move = -move;

        resetEncoders();
        frontLeft.setTargetPosition(move);
        frontRight.setTargetPosition(move);
        backLeft.setTargetPosition(move);
        backRight.setTargetPosition(move);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setAllPower(DRIVE_SPEED);
        runtime.reset();
        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) &&
                runtime.seconds() < 5) {
            telemetry.addData("Driving", inches);
            telemetry.update();
        }
        stopAll();
        sleep(100);
    }

    private void strafing(double inches, boolean left) {
        double correction = 1.12;
        int move = (int) (inches * TICKS_PER_INCH * correction);

        resetEncoders();

        if (left) {
            frontLeft.setTargetPosition(-move);
            frontRight.setTargetPosition(move);
            backLeft.setTargetPosition(move);
            backRight.setTargetPosition(-move);
        } else {
            frontLeft.setTargetPosition(move);
            frontRight.setTargetPosition(-move);
            backLeft.setTargetPosition(-move);
            backRight.setTargetPosition(move);
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setAllPower(DRIVE_SPEED);
        runtime.reset();
        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) &&
                runtime.seconds() < 5) {
            telemetry.addData("Strafing", inches);
            telemetry.update();
        }
        stopAll();
        sleep(100);
    }


    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
