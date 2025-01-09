package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="SimbaTeleOp1/8.2", group="TeleOp")
public class SimbaCodeTeleOp extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;
    private DcMotor leftVertMotor;
    private DcMotor rightVertMotor;
    private Servo linearServo;
    private Servo rightClaw;
    private Servo leftClaw;
    final double viperPower = 1.0;
    @Override
    public void runOpMode() {
        // Initialize motors
        leftFrontMotor = hardwareMap.get(DcMotor.class, "Left_front");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "Right_front");
        leftRearMotor = hardwareMap.get(DcMotor.class, "Left_rear");
        rightRearMotor = hardwareMap.get(DcMotor.class, "Right_rear");

        leftVertMotor = hardwareMap.get(DcMotor.class, "Left_vert");
        rightVertMotor = hardwareMap.get(DcMotor.class, "Right_vert");

        linearServo = hardwareMap.get(Servo.class, "linearServo");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightVertMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();

        while (opModeIsActive()) {
            // Drive controls
            // Handle motor control
            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = - gamepad1.left_stick_y;
            double rightStickX = gamepad1.right_stick_x;

            // Calculate wheel powers
            double frontLeftPower = leftStickY + leftStickX;
            double frontRightPower = leftStickY - leftStickX;
            double backLeftPower = leftStickY + rightStickX;
            double backRightPower = leftStickY - rightStickX;

            // Set motor powers
            leftFrontMotor.setPower(frontLeftPower);
            rightFrontMotor.setPower(frontRightPower);
            leftRearMotor.setPower(backLeftPower);
            rightRearMotor.setPower(backRightPower);
            if (gamepad1.right_bumper) {
                leftVertMotor.setPower(viperPower);
                rightVertMotor.setPower(viperPower);
            }
            else if(gamepad1.dpad_down){
                leftVertMotor.setPower(0);
                rightVertMotor.setPower(0);
            }
            else if (gamepad1.left_bumper) {
                leftVertMotor.setPower(-viperPower);
                rightVertMotor.setPower(-viperPower);
            }

            // Linear servo control
            if (gamepad1.a) {
                linearServo.setPosition(1.0); // Fully extended
            } else if (gamepad1.b) {
                linearServo.setPosition(0.0); // Fully retracted
            }

            // Claw control - synced servos
            if (gamepad1.x) {
                leftClaw.setPosition(1.0); // Close claw
                rightClaw.setPosition(-1.0);
            } else if (gamepad1.y) {
                leftClaw.setPosition(-0.5); // Open claw
                rightClaw.setPosition(0.5);
            }
        }
    }
}