package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "KURRY TeleOp", group = "Linear Opmode")
public class KurryTeleOpwithGyroV1 extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor launcherLeft, launcherRight, intake;
    private Servo flapperLeft;
    private Servo flapperRight;
    private CRServo servoWheel;
    private BNO055IMU imu;
    private boolean isHoldingTriggerR = false;
    private boolean isHoldingPositionR = false;
    private boolean isHoldingTriggerL = false;
    private boolean isHoldingPositionL = false;
    private int holdPosition = 0;
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
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        launcherLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flapperLeft.setDirection(Servo.Direction.REVERSE);
        flapperRight.setDirection(Servo.Direction.FORWARD);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flapperLeft.setPosition(flapperPosition);


        telemetry.addLine("KURRY TeleOp Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double axial = -gamepad1.left_stick_y * SPEED_FACTOR;
            double lateral = gamepad1.left_stick_x * SPEED_FACTOR;
            double yaw = gamepad1.right_stick_x * SPEED_FACTOR;

            Orientation angles = imu.getAngularOrientation();
            double currentAngle = angles.firstAngle;
            double botHeading = Math.toRadians(-currentAngle);
            double rotatedLateral = (lateral * Math.cos(botHeading)) - (axial * Math.sin(botHeading));
            double rotatedAxial = (lateral * Math.sin(botHeading)) + (axial * Math.cos(botHeading));

            // Calculate wheel powers
            double frontLeftPower = rotatedAxial + rotatedLateral + yaw;
            double frontRightPower = rotatedAxial - rotatedLateral - yaw;
            double backLeftPower = rotatedAxial - rotatedLateral + yaw;
            double backRightPower = rotatedAxial + rotatedLateral - yaw;

            double maxPower = Math.max(
                    Math.abs(frontLeftPower),
                    Math.max(Math.abs(frontRightPower),
                            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
            ));;

            if (maxPower > 1) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // Launch activation (A button)
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

            //if (gamepad1.b) {
            //  rightLaunchPower += 0.05;
            //if (rightLaunchPower > 1.0) {
            //  rightLaunchPower = 1.0;
            //}
            //}
            //}

            //if (gamepad1.x) {
            //  leftLaunchPower += 0.05;
            //if (leftLaunchPower > 0.1) {
            //  leftLaunchPower = 0.1;
            //}

            // -------------------------------
            // Intake (Gamepad 1)
            // -------------------------------
            if (gamepad1.right_trigger > 0.2) {
                intake.setPower(0.8); //Intake Forward
            } else if (gamepad1.left_trigger > 0.2) {
                intake.setPower(-0.8); //Intake Backward
            } else {
                intake.setPower(0); //Intake Off
            }

            // -------------------------------
            // Flapper Servo (Gamepad 2)
            // -------------------------------
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

            telemetry.addData("Left Launch Power", "%.2f", leftLaunchPower);
            telemetry.addData("Right Launch Power", "%.2f", rightLaunchPower);
            telemetry.addData("Flapper Pos", "%.2f", flapperPosition);
            telemetry.addData("Flapper Pos", "%.2f", flapperRightPosition);
            telemetry.addData("IMU Heading (deg)", "%.2f", angles.firstAngle); // ADDED: Telemetry for IMU
            telemetry.update();

            sleep(50);
        }
    }
}