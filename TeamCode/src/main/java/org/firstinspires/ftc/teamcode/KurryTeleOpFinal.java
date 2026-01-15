package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "--KURRY TeleOp.V67", group = "Linear Opmode")
public class KurryTeleOpFinal extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor launcherLeft, launcherRight, intake;
    private Servo flapperLeft, flapperRight;
    private CRServo servoWheel;

    private boolean isHoldingTriggerR = false;
    private boolean isHoldingPositionR = false;
    private boolean isHoldingTriggerL = false;
    private boolean isHoldingPositionL = false;
    private int holdPosition = 0;

    // Servo constants
    final double ServoWheelRIGHT = 1;
    final double ServoWheelSTOP = 0;
    final double ServoWheelLEFT = -1;

    // Launcher speed presets
    private double leftLaunchPower = 0.4;
    private double rightLaunchPower = 0.4;
    private double leftLauncherPowerMID = 0.48;
    private double rightLauncherPowerMID = 0.55;
    private double leftLauncherXC = 0.72;
    private double rightLauncherXC = 0.8;

    private double flapperLeftPosition = 0.3;
    private double flapperRightPosition = 0.71;

    private static final double SPEED_FACTOR = 0.7;

    @Override
    public void runOpMode() {
        // Hardware mapping
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");

        intake = hardwareMap.get(DcMotor.class, "intake");
        flapperLeft = hardwareMap.get(Servo.class, "fl");
        flapperRight = hardwareMap.get(Servo.class, "fr");
        servoWheel = hardwareMap.get(CRServo.class, "sw");

        // Motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);


        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);


        launcherLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherRight.setDirection(DcMotor.Direction.FORWARD);

        // Servo directions
        flapperLeft.setDirection(Servo.Direction.REVERSE);
        flapperRight.setDirection(Servo.Direction.FORWARD);

        // Zero power behaviors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("KURRY TeleOp Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Telemetry
            telemetry.addData("Left Servo Pos", "%.2f", flapperLeft.getPosition());
            telemetry.addData("Right Servo Pos", "%.2f", flapperRight.getPosition());

            // Drive control (basic tank/mecanum hybrid)
            double leftStickX = gamepad1.left_stick_x * SPEED_FACTOR;
            double leftStickY = -gamepad1.left_stick_y * SPEED_FACTOR;
            double rightStickX = gamepad1.right_stick_x * SPEED_FACTOR;

            double frontLeftPower = leftStickY + leftStickX;
            double frontRightPower = leftStickY - leftStickX;
            double backLeftPower = leftStickY + rightStickX;
            double backRightPower = leftStickY - rightStickX;

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // Launcher control
            double powerR = 0;
            double powerL = 0;

            if (gamepad2.right_trigger> 0.2) powerR = 0.43;
            if (gamepad2.left_trigger>0.2) powerL = 0.43;
            if (gamepad2.left_bumper) powerL = leftLauncherPowerMID;
            if (gamepad2.right_bumper) powerR = rightLauncherPowerMID;
            if (gamepad2.dpad_up) powerL = leftLauncherXC;
            if (gamepad2.y) powerR = rightLauncherXC;

            launcherRight.setPower(powerR);
            launcherLeft.setPower(powerL);

            // Intake control
            if (gamepad1.right_trigger > 0.2) {
                isHoldingTriggerR = true;
                isHoldingPositionR = false;
                intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intake.setPower(0.8);
            } else if (isHoldingTriggerR) {
                isHoldingTriggerR = false;
                isHoldingPositionR = true;
                holdPosition = intake.getCurrentPosition();
                intake.setTargetPosition(holdPosition);
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setPower(0.3);
            }

            if (gamepad1.left_trigger > 0.2) {
                isHoldingTriggerL = true;
                isHoldingPositionL = false;
                intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intake.setPower(-0.8);
            } else if (isHoldingTriggerL) {
                isHoldingTriggerL = false;
                isHoldingPositionL = true;
                holdPosition = intake.getCurrentPosition();
                intake.setTargetPosition(holdPosition);
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setPower(0.8);
            }

            // Flapper control (no sleep, just toggle positions)
            if (gamepad2.b) {
                flapperLeft.setPosition(0.14);
            } else {
                flapperLeft.setPosition(flapperLeftPosition);
            }

            if (gamepad2.dpad_left) {
                flapperRight.setPosition(0.58);
            } else {
                flapperRight.setPosition(flapperRightPosition);
            }

            // Servo wheel control
            if (gamepad2.left_stick_button) {
                servoWheel.setPower(ServoWheelRIGHT);
            } else if (gamepad2.right_stick_button) {
                servoWheel.setPower(ServoWheelLEFT);
            } else if (gamepad2.dpad_down) {
                servoWheel.setPower(ServoWheelSTOP);
            }

            // Telemetry
            telemetry.addData("Left Launch Power", "%.2f", powerL);
            telemetry.addData("Right Launch Power", "%.2f", powerR);
            telemetry.addData("Flapper left Pos", "%.2f", flapperLeft.getPosition());
            telemetry.addData("Flapper right Pos", "%.2f", flapperRight.getPosition());
            telemetry.update();

            sleep(50);
        }
    }
}
