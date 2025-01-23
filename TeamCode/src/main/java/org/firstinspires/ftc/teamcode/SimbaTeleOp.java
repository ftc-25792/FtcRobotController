package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Shrenik's Child", group="TeleOp")
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
    final double ARM_TICKS_PER_DEGREE = 1425.1/ 360;
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    double armPosition = ARM_COLLAPSED_INTO_ROBOT;
    int buttonPressed=0;
    final double ARM_SCORE_IN_HIGH = 46 * ARM_TICKS_PER_DEGREE;
    final double ARM_HANGING = -15 * ARM_TICKS_PER_DEGREE;
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
        leftVertMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) rightRearMotor).setCurrentAlert(3, CurrentUnit.AMPS);

        ((DcMotorEx) leftVertMotor).setCurrentAlert(4, CurrentUnit.AMPS);
        leftVertMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVertMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftVertMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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

                moveslide((int)ARM_SCORE_IN_HIGH);

            } else if (gamepad1.left_bumper) {

                moveslide(20);




                // Linear servo control
            } else if(gamepad1.dpad_down){
                leftVertMotor.setPower(0);
                rightVertMotor.setPower(0);
            }
            if (gamepad1.a) {
                    linearServo.setPosition(1.0); // Fully extended
                } else if (gamepad1.b) {
                    linearServo.setPosition(0.0); // Fully retracted
                }
                // Claw control - synced servos
                if (gamepad1.x) {
                    rightClaw.setPosition(-1.0);


                } else if (gamepad1.y) {
                    rightClaw.setPosition(0.5);

                }
            telemetry.addData("LeftTarget: ", leftVertMotor.getTargetPosition());
            telemetry.addData("RightTArget:", rightVertMotor.getTargetPosition());

            telemetry.addData("Status", "Running");
            telemetry.update();
            }


        }

        void moveslide(int height) {
            telemetry.addData("Slide Value", height);
            telemetry.update();
            rightVertMotor.setTargetPosition(height);
            rightVertMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            leftVertMotor.setTargetPosition(height);
            leftVertMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftVertMotor .setPower(1);
            rightVertMotor.setPower(1);

    }

    }



