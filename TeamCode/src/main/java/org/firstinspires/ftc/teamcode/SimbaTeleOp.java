package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="SimbaTeleOp1/8.3", group="TeleOp")
public class SimbaTeleOp extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;
    private DcMotor leftVertMotor;
    private DcMotor rightVertMotor;
    private Servo linearServo;
    private Servo rightClaw;
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
        rightVertMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();

        while (opModeIsActive()) {
            // Drive controls
            // Handle motor control
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Set motor powers
            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
            leftRearMotor.setPower(leftBackPower);
            rightRearMotor.setPower(rightBackPower);
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
            else if (gamepad1.right_stick_button){
                leftVertMotor.setPower(0);
                rightVertMotor.setPower(0);
            }

            // Linear servo control
            if (gamepad1.a) {
                linearServo.setPosition(1.0); // Fully extended
            } else if (gamepad1.b) {
                linearServo.setPosition(0.0); // Fully retracted
            }

            // Claw control - synced servos
            if (gamepad1.x) {
                rightClaw.setPosition(-1.0);
                updateTelemetry("Claw Postion:",rightClaw.getPosition());
              } else if (gamepad1.y) {
                rightClaw.setPosition(0.5);
                updateTelemetry("Claw Postion:",rightClaw.getPosition());
            }
        }
    }
    public void updateTelemetry(String caption, double data)
    {
        telemetry.addData(caption, data);
        telemetry.update();
    }
}
