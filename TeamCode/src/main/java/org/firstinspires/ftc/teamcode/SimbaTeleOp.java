package org.firstinspires.ftc.teamcode;package.org.firstinspires.ftc.teamcode;

import com.google.blocks.ftcrobotcontroller.runtime.CRServoAccess;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
    private DcMotor armMotor;
    private IMU imu;
    CRServo intake;
    Servo wrist;
    double viperposition = 0.0;
    final double ARM_TICKS_PER_DEGREE = 7125.16 / 360;
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 10 * ARM_TICKS_PER_DEGREE;//250
    final double ARM_SCORE_SAMPLE_IN_LOW = 30 * ARM_TICKS_PER_DEGREE; //160
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE; // Allow for slight adjustments
    final double ARM_SCORE_IN_HIGH = 70 * ARM_TICKS_PER_DEGREE;


    //Viper Slide Control Variables
    final double VIPERSLIDE_TICKS_PER_DEGREE = 2500 / 360;
    final double VIPER_COLLECT = 5 * VIPERSLIDE_TICKS_PER_DEGREE;
    //final double Maxposition = 1800 * VIPERSLIDE_TICKS_PER_DEGREE;
    final double Maxposition = 26 * VIPERSLIDE_TICKS_PER_DEGREE;



    double armPosition = ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;
    final double WRIST_FOLDED_IN = 0.8333;
    final double WRIST_FOLDED_OUT = 0.5;
    final double WRIST_FOLDED_MIN = 0.2;
    final double ARM_CLEAR_BARRIER = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 50 * ARM_TICKS_PER_DEGREE;//160
    final double ARM_ATTACH_HANGING_HOOK = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 0 * ARM_TICKS_PER_DEGREE;//15  //8
    final double INTAKE_COLLECT = 1;
    final double INTAKE_OFF = 0;
    final double INTAKE_DEPOSIT = -1;

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
        //armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        //intake = hardwareMap.get(CRServo.class, "intake");
        //wrist = hardwareMap.get(Servo.class, "wrist");

        // Set motor directions
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);

//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        ((DcMotorEx) armMotor).setCurrentAlert(2,CurrentUnit.AMPS);

//         Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD; //BACK
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();
        double leftStickY = gamepad1.left_stick_y;
        double rightStickY = gamepad1.right_stick_y;

        // Set motor powers (adjust as needed for your drivetrain configuration)

        leftFrontMotor.setPower(leftStickY + rightStickY);
        leftRearMotor.setPower(leftStickY + rightStickY);
        rightFrontMotor.setPower(rightStickY - leftStickY);
        rightRearMotor.setPower(rightStickY - leftStickY);
