package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Launcher Separate Control")
public class LauncherSeparateControl extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight, intake;
    private DcMotorEx launcherLeft, launcherRight;
    private Servo flapperLeft, flapperRight;
    private CRServo servoWheel;


    private double leftPower = 0.45;
    private double rightPower = 0.45;

    private static final double STEP = 0.05;


    private double SPEED_FACTOR = 0.7;

    @Override
    public void runOpMode() {

            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");
            launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");
            launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");

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

                if (gamepad1.dpad_up){
                    leftPower += STEP;
                }
                if (gamepad1.dpad_down){
                    leftPower -= STEP;
                }


                if (gamepad1.y){
                    rightPower += STEP;
                }
                if (gamepad1.a){
                    rightPower -= STEP;
                }


                leftPower = Range.clip(leftPower, 0.0, 1.0);
                rightPower = Range.clip(rightPower, 0.0, 1.0);

                launcherLeft.setPower(leftPower);
                launcherRight.setPower(rightPower);

                telemetry.addData("Left Launcher Power", leftPower);
                telemetry.addData("Right Launcher Power", rightPower);
                telemetry.update();
        }
    }
        }

