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

@Autonomous(name = "KurryAutoFullRed", group = "Linear Opmode")
public class KurryAuto extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor launcherLeft, launcherRight, intake;
    private IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private Servo flapperRight, flapperLeft;
    private CRServo divider;

    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.4;
    private static final double HEADING_THRESHOLD = 2.0;

    @Override
    public void runOpMode() {
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

        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParameters);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        launcherLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flapperRight.setDirection(Servo.Direction.REVERSE);
        flapperRight.setPosition(0.22);

        waitForStart();

//        turn(32,true);
//        shoot();//gotta fix this one
//        sleep(670);
//        turn(32,false);
//        driveStraight(1,true);
//        turn(90,true);
//        //have to add intake
//        driveStraight(1,false);
//        turn(32,true);
//        shoot();
//        turn(32,false);
//        sleep(1500);
    }

    private void moveDivider(boolean isRight) {
        if (!isRight){
            divider.setPower(0.67);
            sleep(300);
        } else {
            divider.setPower(-0.67);
            sleep(300);
        }
    }

    private void driveStraight(double distance, boolean isForward) {
        if (isForward) setMotorPower(DRIVE_SPEED);
        else setMotorPower(-DRIVE_SPEED);
        sleep((long)(distance*1000/DRIVE_SPEED));
        setMotorPower(0);
        launcherRight.setPower(0.65);
        launcherLeft.setPower(0.65);
        intake.setPower(1.0);
        sleep(500);
    }

    private void driveBackward(double distance) {
        setMotorPower(-DRIVE_SPEED);
        sleep((long)(distance*1000/DRIVE_SPEED));
        setMotorPower(0);
        launcherRight.setPower(0.65);
        launcherLeft.setPower(0.65);
        intake.setPower(1.0);
        sleep(500);
    }

    private void strafing(double distance, boolean isLeft) {
        if (isLeft) {
            frontLeft.setPower(-DRIVE_SPEED);
            frontRight.setPower(DRIVE_SPEED);
            backLeft.setPower(DRIVE_SPEED);
            backRight.setPower(-DRIVE_SPEED);
        } else {
            frontLeft.setPower(DRIVE_SPEED);
            frontRight.setPower(-DRIVE_SPEED);
            backLeft.setPower(-DRIVE_SPEED);
            backRight.setPower(DRIVE_SPEED);
        }
        sleep((long)(distance*1000/DRIVE_SPEED));
        setMotorPower(0);
        launcherRight.setPower(0.65);
        launcherLeft.setPower(0.65);
        intake.setPower(1.0);
        sleep(1000);
    }

    private void UpDiagonalStraf(double distance, boolean IsUpperLeft) {
        if (IsUpperLeft) {
            frontRight.setPower(DRIVE_SPEED);
            backLeft.setPower(DRIVE_SPEED);
        } else {
            backRight.setPower(DRIVE_SPEED);
            frontLeft.setPower(DRIVE_SPEED);
        }
        launcherRight.setPower(0.65);
        launcherLeft.setPower(0.65);
        intake.setPower(1.0);
        sleep((long)(distance*1000/DRIVE_SPEED));
        setMotorPower(0);
        sleep(500);
    }

    private void DownDiagonalStraf(double distance, boolean IsLowerLeft) {
        if (IsLowerLeft) {
            backRight.setPower(-DRIVE_SPEED);
            frontLeft.setPower(-DRIVE_SPEED);
        } else {
            frontRight.setPower(-DRIVE_SPEED);
            backLeft.setPower(-DRIVE_SPEED);
        }
        launcherRight.setPower(0.65);
        launcherLeft.setPower(0.65);
        intake.setPower(1.0);
        sleep((long)(distance*1000/DRIVE_SPEED));
        setMotorPower(0);
        sleep(500);
    }

    private void turn(double angle, boolean isLeft) {
        imu.resetYaw();
        double startAngle = getHeading();
        double targetAngle = normalizeAngle(startAngle + (isLeft ? angle : -angle));

        while (opModeIsActive() && !isAngleReached(targetAngle)) {
            double error = normalizeAngle(targetAngle - getHeading());

            // Proportional control: slow down as we approach target
            double scale = (Math.abs(error) > 10) ? 1.0 : 0.5;
            double leftPower = -TURN_SPEED * Math.signum(error) * scale;
            double rightPower = TURN_SPEED * Math.signum(error) * scale;

            frontLeft.setPower(leftPower);
            backLeft.setPower(leftPower);
            frontRight.setPower(rightPower);
            backRight.setPower(rightPower);

            telemetry.addData("Current Angle", getHeading());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Error", error);
            telemetry.update();
        }

        setMotorPower(0); // Stop all motors
        sleep(300); // Pause briefly after turning
    }

    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    private boolean isAngleReached(double targetAngle) {
        double currentAngle = normalizeAngle(getHeading());
        return Math.abs(currentAngle - targetAngle) < HEADING_THRESHOLD;
    }

    private void setMotorPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        launcherRight.setPower(0.65);
        launcherLeft.setPower(0.65);
        intake.setPower(1.0);
    }

    void flapper() {
        flapperRight.setPosition(0.33);
        sleep(1000);
        flapperRight.setPosition(0.67);
        sleep(1000);
        flapperRight.setPosition(0.33);
        sleep(1000);
        flapperLeft.setPosition(0.02);
        sleep(1000);
        flapperLeft.setPosition(0.28);
        sleep(1000);
        launcherLeft.setPower(0.65);
        launcherRight.setPower(0.65);
        intake.setPower(1.0);
    }

    private void shoot(boolean IsRight) {
        if(IsRight) {
            launcherRight.setPower(0.8);
        } else{
            launcherLeft.setPower(0.8);
        }
        sleep(2500);
    }
}
