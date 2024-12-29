package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "SimbaTeleOp", group = "TeleOp")
public class SimbaTeleOp extends LinearOpMode {
    private DcMotor leftFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;
    private DcMotor leftVertMotor;
    private DcMotor rightVertMotor;
    private IMU imu;
    private Servo linear;
    private Servo rightClaw;
    private Servo leftClaw;

    //Viper Slide Control Variables
    final double VERTICAL_MOTION = 0.3;

    //Linear Servo Control
    final double LINEAR_OUT = 1;
    final double LINEAR_IN = 0;

    private static final double FORWARD_DISTANCE = 0.5; // Adjust distance in meters
    private static final double TURN_ANGLE = -90.0; // Degrees to turn
    private static final double DRIVE_SPEED = 0.9; // Speed for driving

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFrontMotor = hardwareMap.get(DcMotor.class, "Left_front");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "Right_front");
        leftRearMotor = hardwareMap.get(DcMotor.class, "Left_rear");
        rightRearMotor = hardwareMap.get(DcMotor.class, "Right_rear");
        leftVertMotor = hardwareMap.get(DcMotor.class, "Left_vert");
        rightVertMotor = hardwareMap.get(DcMotor.class, "Right_vert");
        linear = hardwareMap.get(Servo.class, "linear");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        // Set motor directions
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftVertMotor.setDirection(DcMotor.Direction.FORWARD);
        rightVertMotor.setDirection(DcMotor.Direction.FORWARD);

//         Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP; //BACK
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        waitForStart();
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;
        // Calculate wheel powers, incorporating strafing
        double frontLeftPower = leftStickY + leftStickX - rightStickX;
        double frontRightPower = leftStickY - leftStickX + rightStickX;
        double backLeftPower = leftStickY + rightStickX + leftStickX;
        double backRightPower = leftStickY - rightStickX - leftStickX;
        // Set motor powers
        leftFrontMotor.setPower(frontLeftPower);
        rightFrontMotor.setPower(frontRightPower);
        leftRearMotor.setPower(backLeftPower);
        rightRearMotor.setPower(backRightPower);

        if (gamepad1.dpad_down) {
            linear.setPosition(LINEAR_IN);
        } else if (gamepad1.dpad_up) {
            linear.setPosition(LINEAR_OUT);
        }
        //moves viper slides a small amount ("tick")
        if (gamepad1.right_bumper) {
            verticalMovement(0.6,true);

        } else if (gamepad1.left_bumper) {
            verticalMovement(0.6,false);

        } else if (gamepad1.a) {
            rightVertMotor.setPower(0.0);
            leftVertMotor.setPower(0);
        }

        if (gamepad1.x) {
            claw(1,true);
        } else if (gamepad1.y) {
            claw(1, false);
        }

    }
    private void claw(double position, boolean isOpen){
        if(isOpen == true){
            //Open - Test
            rightClaw.setPosition(position);
            leftClaw.setPosition(-position);
        }else{
            //Close - Test
            rightClaw.setPosition(-position);
            leftClaw.setPosition(position);

        }

    }
    private void verticalMovement(double distance, boolean isUp){
        if(isUp == true){
            rightVertMotor.setPower(VERTICAL_MOTION);
            leftVertMotor.setPower(VERTICAL_MOTION);
        } else {
            rightVertMotor.setPower(-VERTICAL_MOTION);
            leftVertMotor.setPower(-VERTICAL_MOTION);
        }

        sleep((long) (distance * 1000 /VERTICAL_MOTION)); // Adjust time based on speed
        setMotorPower(0); // Stop all motors
        sleep(500); // Pause briefly after moving forward
    }
    private void setMotorPower(double power) {
        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);
    }

}