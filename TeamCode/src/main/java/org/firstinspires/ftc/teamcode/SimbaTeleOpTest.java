package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="SimbaTeleOp1/8", group="TeleOp")
public class SimbaTeleOpTest extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;
    private DcMotor leftVertMotor;
    private DcMotor rightVertMotor;
    private Servo linearServo;
    private Servo rightClaw;
    private Servo leftClaw;

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


        waitForStart();

        while (opModeIsActive()) {
            // Drive controls
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            double frontLeftPower = drive + strafe + rotate;
            double frontRightPower = drive - strafe - rotate;
            double backLeftPower = drive - strafe + rotate;
            double backRightPower = drive + strafe - rotate;

            leftFrontMotor.setPower(frontLeftPower);
            rightFrontMotor.setPower(frontRightPower);
            leftRearMotor.setPower(backLeftPower);
            rightRearMotor.setPower(backRightPower);

            // Sync the viper slides
            if (gamepad1.right_bumper) {
                leftVertMotor.setPower(0.8);
                rightVertMotor.setPower(-0.8);
            }
            else if (gamepad1.left_bumper) {
                leftVertMotor.setPower(-0.8);
                rightVertMotor.setPower(0.8);
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
