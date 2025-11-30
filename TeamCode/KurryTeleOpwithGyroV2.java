package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
// ADDED: Import for the IMU Orientation object
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU; // ADDED: Import for the IMU class

@TeleOp(name = "KURRY TeleOp", group = "Linear Opmode")
public class KurryTeleOpwithGyroV2 extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor launcherLeft, launcherRight, intake;
    private Servo flapperLeft;
    private Servo flapperRight;
    private CRServo servoWheel;
    private BNO055IMU imu; // ADDED: IMU Declaration

    // REMOVED: isHoldingTriggerR, isHoldingPositionR, isHoldingTriggerL, isHoldingPositionL
    // REMOVED: holdPosition (Replaced by simpler BRAKE logic)

    final double ServoWheelRIGHT = 1;
    final double ServoWheelSTOP = 0;
    final double ServoWheelLEFT = -1;
    final double flapperLEFT = 1;
    final double flapperLEFTSTOP = 0;
    final double flapperLEFTDOWN = -1;

    // Launcher speed presets (MID speed is still unused, ready for future toggle logic)
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

        // ADDED: IMU Mapping and Configuration
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
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Motor mode set once for simple power control
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Crucial for simplified stop/hold

        flapperLeft.setPosition(flapperPosition);

        telemetry.addLine("KURRY TeleOp Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 1. Read Raw Driver Inputs
            double axial = -gamepad1.left_stick_y * SPEED_FACTOR;
            double lateral = gamepad1.left_stick_x * SPEED_FACTOR;
            double yaw = gamepad1.right_stick_x * SPEED_FACTOR;

            // 2. IMU Reading and Rotation Calculation (ADDED FOR FIELD-CENTRIC DRIVE)
            Orientation angles = imu.getAngularOrientation();
            double currentAngle = angles.firstAngle;
            double botHeading = Math.toRadians(-currentAngle);

            double rotatedLateral = (lateral * Math.cos(botHeading)) - (axial * Math.sin(botHeading));
            double rotatedAxial = (lateral * Math.sin(botHeading)) + (axial * Math.cos(botHeading));

            // 3. Calculate Wheel Powers (USING ROTATED INPUTS)
            double frontLeftPower = rotatedAxial + rotatedLateral + yaw;
            double frontRightPower = rotatedAxial - rotatedLateral - yaw;
            double backLeftPower = rotatedAxial - rotatedLateral + yaw;
            double backRightPower = rotatedAxial + rotatedLateral - yaw;

            // 4. Normalization (ADDED)
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

            // Launcher activation (FIXED BUG: Combined duplicate/conflicting IF blocks)

            // Left Launcher runs if A OR X is pressed
            if (gamepad2.a || gamepad2.x) {
                launcherLeft.setPower(leftLaunchPower);
            } else {
                launcherLeft.setPower(0);
            }

            // Right Launcher runs if B OR Y is pressed
            if (gamepad2.b || gamepad2.y) {
                launcherRight.setPower(rightLaunchPower);
            } else {
                launcherRight.setPower(0);
            }

            // REMOVED: Obsolete if blocks for gamepad2.x and gamepad2.y

            // -------------------------------
            // Intake (Gamepad 1) - SIMPLIFIED LOGIC
            // REMOVED: Complex flag logic and RUN_TO_POSITION
            // -------------------------------
            if (gamepad1.right_trigger > 0.2) {
                // Intake In (Forward)
                intake.setPower(0.8);
            } else if (gamepad1.left_trigger > 0.2) {
                // Intake Out (Reverse)
                intake.setPower(-0.8);
            } else {
                // Stop/Hold - Uses BRAKE behavior
                intake.setPower(0.0);
            }
            // REMOVED: All the original else if (isHolding...) blocks

            // -------------------------------
            // Flapper Servo (Gamepad 2) - UNCHANGED
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