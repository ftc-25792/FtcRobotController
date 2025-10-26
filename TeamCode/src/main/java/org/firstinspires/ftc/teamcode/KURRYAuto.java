package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "KurryAuto", group = "Linear Opmode")
public class KurryAUTO extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor launcherLeft, launcherRight, intake;
    private ColorSensor colorSensor;
    private IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private String[] ballColors = new String[3];
    private int detectedCount = 0;
    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.4;
    private static final double HEADING_THRESHOLD = 2.0; // degrees tolerance

    @Override
    public void runOpMode() {

        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Initialize color sensor
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        // Initialize IMU (Control Hub built-in gyro)
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParameters);

        telemetry.addLine("IMU initializing...");
        telemetry.update();
        sleep(1000);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        launcherLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialized — Ready to Start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            detectThreeBalls();

            telemetry.addData("Detected Balls", "%s, %s, %s",
                    ballColors[0], ballColors[1], ballColors[2]);
            telemetry.update();
            sleep(1000);

            String combo = ballColors[0] + ballColors[1] + ballColors[2];

            if (combo.equals("PurplePurpleGreen")) {
                purplePurpleGreen();
            } else if (combo.equals("PurpleGreenPurple")) {
                purpleGreenPurple();
            } else if (combo.equals("GreenPurplePurple")) {
                greenPurplePurple();
            } else {
                telemetry.addLine("Unknown combo. Fallback drive.");
                telemetry.update();
                driveBackward(0.4, 800);
            }

            sleep(2000);
        }
    }
    private void detectThreeBalls() {
        detectedCount = 0;
        while (opModeIsActive() && detectedCount < 3) {
            intake.setPower(0.7);
            runtime.reset();

            while (opModeIsActive() && runtime.seconds() < 4) {
                int r = colorSensor.red();
                int g = colorSensor.green();
                int b = colorSensor.blue();
                int alpha = colorSensor.alpha();

                if (alpha > 100) {
                    String color = detectColorFromRGB(r, g, b);
                    ballColors[detectedCount] = color;
                    telemetry.addData("Ball %d", detectedCount + 1);
                    telemetry.addData("Detected Color", color);
                    telemetry.update();
                    detectedCount++;
                    intake.setPower(0);
                    sleep(1000);
                    break;
                }
            }
        }
        intake.setPower(0);
    }

    private String detectColorFromRGB(int r, int g, int b) {
        if (r > b && r > g) return "Purple"; // purple = red+blue dominant
        if (g > r && g > b) return "Green";
        if (b > r && b > g) return "Purple";
        return "Unknown";
    }
    private void purplePurpleGreen() {
        telemetry.addLine("Running Routine: Purple, Purple, Green");
        telemetry.update();
        driveForward(0.5, 800);
        launchBall();
        driveBackward(0.4, 400);
    }

    private void purpleGreenPurple() {
        telemetry.addLine("Running Routine: Purple, Green, Purple");
        telemetry.update();
        turnRight(90);
        launchBall();
        turnLeft(90);
    }
    private void greenPurplePurple() {
        telemetry.addLine("Running: Green, Purple, Purple");
        telemetry.update();
        driveForward(0.4, 600);
        turnLeft(45);
        driveForward(0.4, 400);
    }

    private void driveForward(double power, long timeMs) {
        setAllPower(power);
        sleep(timeMs);
        setAllPower(0);
    }

    private void driveBackward(double power, long timeMs) {
        setAllPower(-power);
        sleep(timeMs);
        setAllPower(0);
    }

    private void setAllPower(double p) {
        frontLeft.setPower(p);
        frontRight.setPower(p);
        backLeft.setPower(p);
        backRight.setPower(p);
    }
    private void launchBall() {
        telemetry.addLine("Launching Ball...");
        telemetry.update();
        launcherLeft.setPower(1);
        launcherRight.setPower(1);
        sleep(1000);
        launcherLeft.setPower(0);
        launcherRight.setPower(0);
    }
    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private boolean isAngleReached(double targetAngle) {
        double current = getHeading();
        double error = targetAngle - current;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return Math.abs(error) < HEADING_THRESHOLD;
    }

    private void turnLeft(double degrees) {
        turnToAngle(-degrees);
    }

    private void turnRight(double degrees) {
        turnToAngle(degrees);
    }

    private void turnToAngle(double targetAngle) {
        double startAngle = getHeading();
        double desiredAngle = startAngle + targetAngle;

        while (desiredAngle >= 180) desiredAngle -= 360;
        while (desiredAngle < -180) desiredAngle += 360;

        telemetry.addData("Turning", "Target: %.1f°", desiredAngle);
        telemetry.update();

        boolean turnRight = targetAngle > 0;
        double turnPower = TURN_SPEED;

        if (turnRight) {
            frontLeft.setPower(turnPower);
            backLeft.setPower(turnPower);
            frontRight.setPower(-turnPower);
            backRight.setPower(-turnPower);
        } else {
            frontLeft.setPower(-turnPower);
            backLeft.setPower(-turnPower);
            frontRight.setPower(turnPower);
            backRight.setPower(turnPower);
        }

        while (opModeIsActive() && !isAngleReached(desiredAngle)) {
            telemetry.addData("Heading", getHeading());
            telemetry.addData("Target", desiredAngle);
            telemetry.update();
        }

        setAllPower(0);
        sleep(300);
    }
}
