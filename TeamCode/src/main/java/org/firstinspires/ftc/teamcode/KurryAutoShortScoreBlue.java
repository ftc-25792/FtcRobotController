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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "KurryAutoShortScoreB", group = "Linear Opmode")
public class KurryAutoShortScoreBlue extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor launcherLeft, launcherRight, intake;
    private IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private Servo flapperRight, flapperLeft;
    private CRServo divider;

    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.5;
    static final double HEADING_THRESHOLD = 1.0;

    static final double WHEEL_DIAMETER_MM = 104;
    static final double WHEEL_DIAMETER_IN = WHEEL_DIAMETER_MM / 25.4;
    static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_IN;

    static final double TICKS_PER_REV = 384.5;
    static final double TICKS_PER_INCH = TICKS_PER_REV / WHEEL_CIRCUMFERENCE;

    @Override
    public void runOpMode() {
        // Hardware mapping
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flapperRight = hardwareMap.get(Servo.class, "fr");
        flapperLeft = hardwareMap.get(Servo.class, "fl");
        divider = hardwareMap.get(CRServo.class, "sw");
        imu = hardwareMap.get(IMU.class, "imu");

        // IMU initialization
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu.initialize(imuParameters);

        // Motor directions
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        launcherLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);

        flapperRight.setDirection(Servo.Direction.REVERSE);
        flapperRight.setPosition(0.22);

        waitForStart();

        resetEncoders();

        // Example autonomous routine
        driveStraight(30, true);
        turn(45, false);





//        WheelsOn;
//        driveStraight(15,false);
//        shoot;
//        driveStraight(30,false);
//        turn(45, true);

//        intakeIn;
//        driveStraight(36,true);
//          sorting wheel;
//          driveStraight(36,false);
//          turn(45, false);
//          driveStraight(36,true);
//          shoot;
//          turn(45,false);
//          driveStraight(20,false);
    }

    // ------------------ Launchers ------------------
    private void launch(double p) {
        if (p == 1) {
            flapperLeft.setPosition(0.3);
            sleep(1000);
            flapperLeft.setPosition(0.14);
            sleep(1000);
            flapperRight.setPosition(0.71);
            sleep(1000);
            flapperRight.setPosition(0.58);
            sleep(1000);
            flapperRight.setPosition(0.71);
            sleep(1000);
            flapperRight.setPosition(0.58);
        }
    }

    // ------------------ Encoder Methods ------------------
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

    // ------------------ Drive Methods ------------------
    private void driveStraight(double inches, boolean isForward) {
        int move = (int)(inches * TICKS_PER_INCH);
        if (!isForward) move = -move;

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

    private void strafing(double inches, boolean isLeft) {
        double correction = 1.12;
        int move = (int)(inches * TICKS_PER_INCH * correction);

        resetEncoders();

        if (isLeft) {
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
    private void turn(double angle, boolean isLeft) {
        imu.resetYaw();
        double startAngle = getHeading();
        double targetAngle = startAngle + angle;

        // Normalize the angle
        while (targetAngle >= 180) targetAngle -= 360;
        while (targetAngle < -180) targetAngle += 360;

        // Set motors for turning
        if (isLeft) {
            frontLeft.setPower(-TURN_SPEED);
            backLeft.setPower(-TURN_SPEED);
            frontRight.setPower(TURN_SPEED);
            backRight.setPower(TURN_SPEED);
        } else {
            frontLeft.setPower(TURN_SPEED);
            backLeft.setPower(TURN_SPEED);
            frontRight.setPower(-TURN_SPEED);
            backRight.setPower(-TURN_SPEED);
        }

        // Continue turning until the target angle is reached
        while (opModeIsActive() && !isAngleReached(targetAngle)) {
            telemetry.addData("Current Angle", getHeading());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.update();
        }

        setMotorPower(0); // Stop all motors
        sleep(500); // Pause briefly after turning
    }


    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private boolean isAngleReached(double targetAngle) {
        double currentAngle = getHeading();
        // Normalize current angle
        while (currentAngle >= 180) currentAngle -= 360;
        while (currentAngle < -180) currentAngle += 360;
        return Math.abs(currentAngle - targetAngle) < HEADING_THRESHOLD; // Tolerance check
    }

    private void setMotorPower(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    // Method to get steering correction
    private double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        double headingError = desiredHeading - getHeading();

        // Normalize the error
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return headingError * proportionalGain; // Proportional controller
    }
}
//    // ------------------ Gyro Turn ------------------
//    private void turn(double targetAngle, boolean isLeft) {
//        double startHeading = getHeading();
//        double desiredHeading = normalizeAngle(startHeading + (isLeft ? targetAngle : -targetAngle));
//        double error = getHeadingError(desiredHeading);
//
//        while (opModeIsActive() && Math.abs(error) > HEADING_THRESHOLD) {
//            error = getHeadingError(desiredHeading);
//
//            double kP = 0.01;  // Proportional constant
//            double turnPower = Range.clip(error * kP, -TURN_SPEED, TURN_SPEED);
//
//            // Mecanum in-place turn
//            frontLeft.setPower(turnPower);
//            backLeft.setPower(turnPower);
//            frontRight.setPower(-turnPower);
//            backRight.setPower(-turnPower);
//
//            telemetry.addData("Target", desiredHeading);
//            telemetry.addData("Current", getHeading());
//            telemetry.addData("Error", error);
//            telemetry.addData("TurnPower", turnPower);
//            telemetry.update();
//        }
//
//        stopAll();
//        sleep(150);
//    }
//
//    // ------------------ IMU Helpers ------------------
//    private double getHeading() {
//        YawPitchRollAngles yp = imu.getRobotYawPitchRollAngles();
//        return yp.getYaw(AngleUnit.DEGREES);
//    }
//
//    private double getHeadingError(double targetAngle) {
//        double robotHeading = getHeading();
//        double error = targetAngle - robotHeading;
//        return normalizeAngle(error);
//    }
//
//    private double normalizeAngle(double angle) {
//        while (angle > 180) angle -= 360;
//        while (angle <= -180) angle += 360;
//        return angle;
//    }


