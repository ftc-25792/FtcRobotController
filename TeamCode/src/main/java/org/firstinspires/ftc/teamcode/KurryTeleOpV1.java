package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// --- MODERN IMU IMPORTS ---
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "KURRY TeleOp Gyro", group = "Linear Opmode")
public class KurryTeleOpwithGyroV1 extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor launcherLeft, launcherRight, intake;
    private Servo flapperLeft;
    private Servo flapperRight;
    private CRServo servoWheel;
    private IMU imu;
    private int holdPosition = 0;

    // --- TOGGLE VARIABLES ---
    private boolean isFieldCentric = true; // Start in Field Centric mode
    private boolean lastToggleState = false; // Debouncing variable for the toggle switch
    // ------------------------

    final double ServoWheelRIGHT = 1;
    final double ServoWheelSTOP = 0;
    final double ServoWheelLEFT = -1;
    final double flapperLEFT = 1;
    final double flapperLEFTSTOP = 0;
    final double flapperLEFTDOWN = -1;

    // Launcher speed presets
    private double leftLaunchPower = 0.4;
    private double rightLaunchPower = 0.55;
    private double leftLauncherPowerMID = 0.80;
    private double rightLauncherPowerMID = 0.80;

    private double flapperLeftPosition = 0.3; // Left Flapper Default
    private double flapperRightPosition = 0.71; // Right Flapper Default

    private static final double SPEED_FACTOR = 0.7;

    @Override
    public void runOpMode() {
        // Hardware Map Initialization
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flapperLeft = hardwareMap.get(Servo.class, "fl");
        flapperRight = hardwareMap.get(Servo.class, "fr");
        servoWheel = hardwareMap.get(CRServo.class, "sw");

        // IMU Configuration
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);
        imu.initialize(parameters);
        imu.resetYaw();

        // Motor/Servo Directions and Behaviors
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        launcherLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flapperLeft.setDirection(Servo.Direction.REVERSE);
        flapperRight.setDirection(Servo.Direction.FORWARD);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("KURRY TeleOp Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- 0. MODE TOGGLE (Single Button: Back) ---
            boolean currentToggleState = gamepad1.back;

            if (currentToggleState && !lastToggleState) {
                isFieldCentric = !isFieldCentric;

                if (isFieldCentric) {
                    imu.resetYaw();
                }
            }
            lastToggleState = currentToggleState;
            // ------------------------------------------------------

            telemetry.addData("Left Servo Pos", "%.2f",flapperLeft.getPosition());
            telemetry.addData("Right Servo Pos", "%.2f",flapperRight.getPosition());

            // 1. Read Raw Driver Inputs
            double axial = -gamepad1.left_stick_y * SPEED_FACTOR;
            double lateral = gamepad1.left_stick_x * SPEED_FACTOR;
            double yaw = gamepad1.right_stick_x * SPEED_FACTOR;

            // 2. IMU Reading and Rotation Calculation
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double currentAngle = orientation.getYaw(AngleUnit.DEGREES);
            double botHeading = Math.toRadians(-currentAngle);
            double pitch = orientation.getPitch(AngleUnit.DEGREES);
            double roll = orientation.getRoll(AngleUnit.DEGREES);

            double rotatedAxial, rotatedLateral;

            if (isFieldCentric) {
                // FIELD CENTRIC
                rotatedLateral = (lateral * Math.cos(botHeading)) - (axial * Math.sin(botHeading));
                rotatedAxial = (lateral * Math.sin(botHeading)) + (axial * Math.cos(botHeading));
            } else {
                // ROBOT CENTRIC (OVERRIDE)
                rotatedLateral = lateral;
                rotatedAxial = axial;
            }

            // 3. Calculate Wheel Powers
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

            // Subsystem Control
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

            if (gamepad1.dpad_left) {
                rightLaunchPower +=0.01;
            } else if (gamepad1.dpad_right) {
                rightLaunchPower -= 0.01;
            }
            if (gamepad1.dpad_down) {
                leftLaunchPower += 0.01;
            } else if (gamepad1.dpad_up) {
                leftLaunchPower-= 0.01;
            }

            // Intake
            if (gamepad1.right_trigger > 0.2) {
                intake.setPower(0.8);
            } else if (gamepad1.left_trigger > 0.2) {
                intake.setPower(-0.8);
            } else {
                intake.setPower(0.0);
            }

            // Flapper Servo
            if (gamepad1.x) {
                flapperLeftPosition = 0.14;
                flapperLeftPosition = Math.max(0.0, Math.min(1.0, flapperLeftPosition));
                flapperLeft.setPosition(flapperLeftPosition);
                sleep(1000);
                flapperLeftPosition = 0.3;
                flapperLeftPosition = Math.max(0.0, Math.min(1.0, flapperLeftPosition));
                flapperLeft.setPosition(flapperLeftPosition);
            }

            if (gamepad1.a) {
                flapperRightPosition = 0.58;
                flapperRightPosition = Math.max(0.0, Math.min(1.0, flapperRightPosition));
                flapperRight.setPosition(flapperRightPosition);
                sleep(1000);
                flapperRightPosition =0.71;
                flapperRightPosition = Math.max(0.0, Math.min(1.0, flapperRightPosition));
                flapperRight.setPosition(flapperRightPosition);
            }

            // CRServo (Original Logic)
            if (gamepad1.right_bumper) {
                servoWheel.setPower(ServoWheelRIGHT);
            } else if (gamepad1.left_bumper) {
                servoWheel.setPower(ServoWheelLEFT);
            } else if (gamepad1.right_stick_button) {
                servoWheel.setPower(ServoWheelSTOP);
            }

            flapperLeftPosition = Math.max(0.0, Math.min(1.0, flapperLeftPosition));
            flapperLeft.setPosition(flapperLeftPosition);
            flapperRightPosition = Math.max(0.0, Math.min(1.0, flapperRightPosition));
            flapperRight.setPosition(flapperRightPosition);

            // Telemetry
            telemetry.addData("Drive Mode", isFieldCentric ? "FIELD CENTRIC" : "ROBOT CENTRIC");
            telemetry.addData("Left Launch Power", "%.2f", leftLaunchPower);
            telemetry.addData("Right Launch Power", "%.2f", rightLaunchPower);
            telemetry.addData("Flapper left Pos", "%.2f", flapperLeftPosition);
            telemetry.addData("Flapper right Pos", "%.2f", flapperRightPosition);
            telemetry.addData("Yaw (Heading)", "%.2f", currentAngle);
            telemetry.addData("Pitch", "%.2f", pitch);
            telemetry.addData("Roll", "%.2f", roll);
            telemetry.update();

            sleep(50);
        }
    }
}
