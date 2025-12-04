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


@Autonomous(name = "RebootAutoFinal", group = "Linear Opmode")
public class RedRightGoalStarting extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor launcherRight, launcherLeft, intake;
    private Servo flapperRight;
    private CRServo servowheel;
    private IMU imu;


    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.4;
    private static final double HEADING_THRESHOLD = 2.0;
    private double STRAFE_DISTANCE = 0.0;
    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flapperRight = hardwareMap.get(Servo.class, "flapperRight");
        servowheel = hardwareMap.get(CRServo.class, "servowheel");
        launcherLeft =  hardwareMap.get(DcMotor.class, "launcherLeft");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParameters);

        // --- Motor Directions ---
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        flapperRight.setDirection(Servo.Direction.FORWARD);
        flapperRight.setPosition(0.22);

        waitForStart();

        if (opModeIsActive()) {
            driveStraight(1.5, false);
            flapper();
            launcherRight.setPower(0.65);
            launcherLeft.setPower(0.65);
            sleep(1000);
            launcherRight.setPower(0);
            turn(90, false);

            for (int i = 0; i < 3; i++, STRAFE_DISTANCE =+ 0.5) {
                driveStraight(0.2, true);
                intakeSamples(1, 1.0);
                intakeSamples(1, 1.0);
                driveStraight(0.2, false);
                turn(90,true);
                servowheel.setPower(1);
                flapper();
                launcherRight.setPower(0.70);
                sleep(500);
                launcherRight.setPower(0);
                turn(90,true);
                strafing(STRAFE_DISTANCE, true);
            }
        }
    }

    private void driveStraight(double distance, boolean isForward) {
        if (isForward) setMotorPower(DRIVE_SPEED);
        else setMotorPower(-DRIVE_SPEED);
        sleep((long) (distance * 1000 / DRIVE_SPEED));
        setMotorPower(0);
        launcherRight.setPower(0.65);
        intake.setPower(1.0);
        sleep(500);
    }

    private void strafing(double distance, boolean isRight) {
        if (isRight) {
            frontLeft.setPower(DRIVE_SPEED);
            frontRight.setPower(-DRIVE_SPEED);
            backLeft.setPower(-DRIVE_SPEED);
            backRight.setPower(DRIVE_SPEED);
        } else {
            frontLeft.setPower(-DRIVE_SPEED);
            frontRight.setPower(DRIVE_SPEED);
            backLeft.setPower(DRIVE_SPEED);
            backRight.setPower(-DRIVE_SPEED);
        }
        sleep((long) (distance * 1000 / DRIVE_SPEED));
        setMotorPower(0);
        launcherRight.setPower(0.65);
        intake.setPower(1.0);
        sleep(1000);
    }

    private void turn(double angle, boolean isLeft) {
        imu.resetYaw();
        double startAngle = getHeading();
        double targetAngle = startAngle + angle;
        while (targetAngle >= 180) targetAngle -= 360;
        while (targetAngle < -180) targetAngle += 360;

        if (isLeft) {
            frontLeft.setPower(-TURN_SPEED);
            frontRight.setPower(-TURN_SPEED);
            backLeft.setPower(TURN_SPEED);
            backRight.setPower(TURN_SPEED);
        } else {
            frontLeft.setPower(TURN_SPEED);
            frontRight.setPower(TURN_SPEED);
            backLeft.setPower(-TURN_SPEED);
            backRight.setPower(-TURN_SPEED);
        }

        while (opModeIsActive() && !isAngleReached(targetAngle)) {
            telemetry.addData("Current Angle", getHeading());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.update();
        }

        setMotorPower(0);
        launcherRight.setPower(0.65);
        intake.setPower(1.0);
        sleep(500);
    }

    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private boolean isAngleReached(double targetAngle) {
        double currentAngle = getHeading();
        while (currentAngle >= 180) currentAngle -= 360;
        while (currentAngle < -180) currentAngle += 360;
        return Math.abs(currentAngle - targetAngle) < HEADING_THRESHOLD;
    }

    private void setMotorPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    private void intakeSamples(double seconds, double power) {
        intake.setPower(power);
        sleep((long) (seconds * 1000));
        intake.setPower(0);
    }

    private void flapper() {
        launcherRight.setPower(0.65);
        flapperRight.setPosition(0.22);
        sleep(1000);
        flapperRight.setPosition(0.42);
        sleep(1000);
        flapperRight.setPosition(0.22);
        intake.setPower(1.0);
    }
}
