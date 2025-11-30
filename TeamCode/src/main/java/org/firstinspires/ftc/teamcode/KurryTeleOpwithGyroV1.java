package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//MODERN IMU IMPORTS
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "KURRY TeleOp", group = "Linear Opmode")
public class KurryTeleOpwithGyroV1 extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor launcherLeft, launcherRight, intake;
    private Servo flapperLeft;
    private Servo flapperRight;
    private CRServo servoWheel;
    private IMU imu; // Using the modern IMU interface

    final double ServoWheelRIGHT = 1;
    final double ServoWheelSTOP = 0;
    final double ServoWheelLEFT = -1;
    final double flapperLEFT = 1;
    final double flapperLEFTSTOP = 0;
    final double flapperLEFTDOWN = -1;

    // Launcher speed presets
    private double leftLaunchPower = 0.55;
    private double rightLaunchPower = 0.55;
    private double leftLauncherPowerMID = 0.80;
    private double rightLauncherPowerMID = 0.80;

    private double flapperPosition = 0.02; 
    private double flapperRightPosition = 1;

    private static final double SPEED_FACTOR = 0.7;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flapperLeft = hardwareMap.get(Servo.class, "servo1"); 
        flapperRight = hardwareMap.get(Servo.class, "servo2");
        servoWheel = hardwareMap.get(CRServo.class, "servo3");

        // IMU Configuration
        imu = hardwareMap.get(IMU.class, "imu");

        // Define Hub Orientation: Logo UP, USB FORWARD
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        // Initialize IMU with parameters and reset Yaw (Heading)
        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);
        imu.initialize(parameters);
        imu.resetYaw();
        // ----------------------------------------

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        launcherLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flapperLeft.setDirection(Servo.Direction.REVERSE); // Set direction for left flapper
        flapperRight.setDirection(Servo.Direction.FORWARD); // Set direction for right flapper

        // Motor Brake Behavior and Mode
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flapperLeft.setPosition(flapperPosition); // Initial position for the left flapper

        telemetry.addLine("KURRY TeleOp Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 1. Read Raw Driver Inputs
            double axial = -gamepad1.left_stick_y * SPEED_FACTOR;
            double lateral = gamepad1.left_stick_x * SPEED_FACTOR;
            double yaw = gamepad1.right_stick_x * SPEED_FACTOR;

            // 2. IMU Reading and Rotation Calculation
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            // Get all three angles for display and drive control
            double currentAngle = orientation.getYaw(AngleUnit.DEGREES); // Yaw is the heading for field-centric drive
            double botHeading = Math.toRadians(-currentAngle);
            double pitch = orientation.getPitch(AngleUnit.DEGREES);
            double roll = orientation.getRoll(AngleUnit.DEGREES);

            double rotatedLateral = (lateral * Math.cos(botHeading)) - (axial * Math.sin(botHeading));
            double rotatedAxial = (lateral * Math.sin(botHeading)) + (axial * Math.cos(botHeading));

            // 3. Calculate Wheel Powers (Field-Centric)
            double frontLeftPower = rotatedAxial + rotatedLateral + yaw;
            double frontRightPower = rotatedAxial - rotatedLateral - yaw;
            double backLeftPower = rotatedAxial - rotatedLateral + yaw;
            double backRightPower = rotatedAxial + rotatedLateral - yaw;

            // 4. Normalization
            double maxPower = Math.max(
                    Math.abs(frontLeftPower),
                    Math.max(Math.abs(frontRightPower),
                            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
                    )
            );

            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // Launcher activation (FIXED BUG: Combined A/X and B/Y logic)
            if (gamepad2.a || gamepad2.x) {
                launcherLeft.setPower(leftLaunchPower);
            } else {
                launcherLeft.setPower(0);
            }

            if (gamepad2.b || gamepad2.y) {
                launcherRight.setPower(rightLaunchPower);
            } else {
                launcherRight.setPower(0);
            }

            // Intake (SIMPLIFIED LOGIC: if/else if/else using BRAKE)
            if (gamepad1.right_trigger > 0.2) {
                intake.setPower(0.8);
            } else if (gamepad1.left_trigger > 0.2) {
                intake.setPower(-0.8);
            } else {
                intake.setPower(0.0);
            }

            // Flapper Servo (Standard Servo Control)
            if (gamepad2.dpad_down) {
                flapperPosition = 0.02;
            } else if (gamepad2.dpad_up) {
                flapperPosition = 0.28;
            }

            if (gamepad2.dpad_right) {
                flapperRight.setPosition(0.67);
            } else if (gamepad2.dpad_left) {
                flapperRight.setPosition(0.33);
            }

            
            if (gamepad2.right_stick_button) {
                servoWheel.setPower(ServoWheelRIGHT);
            } else if (gamepad2.left_stick_button) {
                servoWheel.setPower(ServoWheelLEFT);
            } else if (gamepad2.right_bumper) {
                servoWheel.setPower(ServoWheelSTOP);
            }

            flapperPosition = Math.max(0.0, Math.min(1.0, flapperPosition));
            flapperLeft.setPosition(flapperPosition);

            telemetry.addData("Launch Power", "%.2f", leftLaunchPower);
            telemetry.addData("Flapper Pos", "%.2f", flapperPosition);
            // Telemetry for all three rotational angles
            telemetry.addData("Yaw (Heading)", "%.2f", currentAngle);
            telemetry.addData("Pitch", "%.2f", pitch);
            telemetry.addData("Roll", "%.2f", roll);
            telemetry.update();

            sleep(50);
        }
    }
}
