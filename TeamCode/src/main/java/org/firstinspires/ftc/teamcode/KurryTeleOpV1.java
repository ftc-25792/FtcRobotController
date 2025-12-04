package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "KURRY TeleOp", group = "Linear Opmode")
public class KurryTeleOpV1 extends LinearOpMode {
//11/30/25
    //first We made both 1-there is a small difference in trajectory of balls :(
    // We tested the midranfe with 0.55
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor launcherLeft, launcherRight, intake;
    private Servo flapperLeft;
    private Servo flapperRight;
    private CRServo servoWheel;
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
    private double leftLaunchPower = 0.4;
    private double rightLaunchPower = 0.55;
    private double leftLauncherPowerMID = 0.80;
    private double rightLauncherPowerMID = 0.80;

    private double flapperLeftPosition = 0.3;
    private double flapperRightPosition = 0.71;

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
        flapperLeft = hardwareMap.get(Servo.class, "fl");
        flapperRight = hardwareMap.get(Servo.class, "fr");
        servoWheel = hardwareMap.get(CRServo.class, "sw");

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

        telemetry.addLine("KURRY TeleOp Initialized");

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Left Servo Pos", "%.2f",flapperLeft.getPosition());
            telemetry.addData("Right Servo Pos", "%.2f",flapperRight.getPosition());
            double axial = -gamepad1.left_stick_y * SPEED_FACTOR;
            double lateral = gamepad1.left_stick_x * SPEED_FACTOR;
            double yaw = gamepad1.right_stick_x * SPEED_FACTOR;

            // Calculate wheel powers
            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
            if (gamepad2.a) {
                launcherLeft.setPower(leftLaunchPower);
            } else {
                launcherLeft.setPower(0);
            }
            if (gamepad2.b) {
                launcherRight.setPower(rightLaunchPower);
            } else {
                launcherRight.setPower(0);
            }
            if (gamepad1.dpad_left){
                rightLaunchPower +=0.01;
            }else if(gamepad1.dpad_right){
                rightLaunchPower -= 0.01;
            }
            if(gamepad1.dpad_down){
                leftLaunchPower += 0.01;
            } else if(gamepad1.dpad_up){
                leftLaunchPower-= 0.01;
            } else if (gamepad1.right_trigger > 0.2) {

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
            if (gamepad1.x) {
                flapperLeftPosition = 0.3;
            }else if(gamepad1.b){
                flapperLeftPosition = 0.14;
            }
            if (gamepad1.a) {
                flapperRightPosition = 0.58;
            }
            else if(gamepad1.y){
                flapperRightPosition =0.71;
            }

            if (gamepad1.right_bumper) {
                servoWheel.setPower(ServoWheelRIGHT);

            } else if (gamepad1.left_bumper) {
                servoWheel.setPower(ServoWheelLEFT);
            }
            else if (gamepad1.right_stick_button){
                servoWheel.setPower(ServoWheelSTOP);
            }
            flapperLeftPosition = Math.max(0.0, Math.min(1.0, flapperLeftPosition));
            flapperLeft.setPosition(flapperLeftPosition);
            flapperRightPosition = Math.max(0.0, Math.min(1.0, flapperRightPosition));
            flapperRight.setPosition(flapperRightPosition);
            telemetry.addData("Left Launch Power", "%.2f", leftLaunchPower);
            telemetry.addData("Right Launch Power", "%.2f", rightLaunchPower);
            telemetry.addData("Flapper left Pos", "%.2f", flapperLeftPosition);
            telemetry.addData("Flapper right Pos", "%.2f", flapperRightPosition);
            telemetry.update();

            sleep(50);
        }
    }
}