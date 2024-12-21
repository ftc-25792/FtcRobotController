package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Meet2Code_12_11", group="Linear Opmode")
public class Meet2CodeCopy extends LinearOpMode {
    // Declare motors
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor armMotor;
    CRServo intake; // Intaking servo
    Servo wrist; // Wrist servo
    DcMotor viperMotor;

    // Arm control variables
    double viperposition = 0.0;
    final double ARM_TICKS_PER_DEGREE = 7125.16 / 360;
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 10 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW = 30 * ARM_TICKS_PER_DEGREE;
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE; // Allow for slight adjustments
    final double ARM_SCORE_IN_HIGH = 70 * ARM_TICKS_PER_DEGREE;


    //Viper Slide Control Variables
    final double VIPERSLIDE_TICKS_PER_DEGREE = 2500 / 360;
    final double VIPER_COLLECT = 5 * VIPERSLIDE_TICKS_PER_DEGREE;
    //final double Maxposition = 1800 * VIPERSLIDE_TICKS_PER_DEGREE;
    final double Maxposition = 26 * VIPERSLIDE_TICKS_PER_DEGREE;
    double viperPositionFudgeFactor;

    // Define encoder limits
    final int VIPER_SLIDE_MIN_LIMIT = 0;// Minimum position (fully retracted)
    final int VIPER_SLIDE_MAX_LIMIT = 1500;// Maximum position (fully extended to 42 inches)



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
    int extentionLimit = 0;

    @Override
    public void runOpMode() {
        // Initialize motors
        motorFrontLeft = hardwareMap.get(DcMotor.class, "Left_front");
        motorFrontRight = hardwareMap.get(DcMotor.class, "Right_front");
        motorBackLeft = hardwareMap.get(DcMotor.class, "Left_rear");
        motorBackRight = hardwareMap.get(DcMotor.class, "Right_rear");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        viperMotor = hardwareMap.get(DcMotor.class, "viperMotor");

        intake = hardwareMap.get(CRServo.class, "Intake");
        wrist = hardwareMap.get(Servo.class, "Wrist");


        // Set motor directions
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set arm motor to use encoders
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) armMotor).setCurrentAlert(2, CurrentUnit.AMPS);


        ((DcMotorEx) viperMotor).setCurrentAlert(4, CurrentUnit.AMPS);
        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // Itialize servos

        wrist.setDirection(Servo.Direction.REVERSE);

        wrist.setPosition(WRIST_FOLDED_OUT); // Folded in position
        intake.setPower(INTAKE_OFF);

        telemetry.addLine("Robot Ready");
        telemetry.update();

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        // Wait for the game driver to press play
        waitForStart();

        while (opModeIsActive()) {
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
            motorFrontLeft.setPower(frontLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackLeft.setPower(backLeftPower);
            motorBackRight.setPower(backRightPower);

            int currentViperPosition = viperMotor.getCurrentPosition();
            int viperMaxLimit = (armPosition == ARM_SCORE_IN_HIGH) ? VIPER_SLIDE_MAX_LIMIT + 500 : VIPER_SLIDE_MAX_LIMIT;


            //Servo control using GamePad1
            if (gamepad1.a) {
                intake.setPower(INTAKE_COLLECT);
                //  wrist.setPosition(0.0);
            } else if (gamepad1.x) {
                intake.setPower(INTAKE_OFF);
                // wrist.setPosition(0.5);
            } else if (gamepad1.b) {
                intake.setPower(INTAKE_DEPOSIT);
                // wrist.setPosition(1.0);
            }
            if(gamepad1.y) {
                wrist.setPosition(0.5);

            }



            // ViperSlide Control with the Encoder
            if (gamepad2.b && currentViperPosition < VIPER_SLIDE_MAX_LIMIT) {
                // Extend viper slide
                viperposition = VIPER_SLIDE_MAX_LIMIT;
            } else if (gamepad2.left_bumper && currentViperPosition > VIPER_SLIDE_MIN_LIMIT) {
                // Retract viper slide
                viperposition = VIPER_SLIDE_MIN_LIMIT;
            } else {
                // Stop the motor if no button is pressed or limits are reached
                viperMotor.setPower(0);
            }


            //GamePad2 Controls
            if (gamepad2.right_bumper) {
                /* This is the intaking/collecting arm position */
                armPosition = ARM_COLLECT;
                //viperPosition = VIPER_COLLECT`;


                wrist.setPosition(0.5);
                //viperMotor.setPower(0.2); // Extend Viper slide
                //intake.setPosition(INTAKE_COLLECT);


                //  double curtime = getRuntime();
                ///  while(true) {
                //   if (getRuntime() > curtime+2)
                //     break;

                //  viperMotor.setPower(-0.5);
                //}
            } else if (gamepad2.dpad_down) {
                armPosition = ARM_WINCH_ROBOT;

            } else if (gamepad2.dpad_right) {
                armPosition = ARM_SCORE_IN_HIGH;

            } else if (gamepad2.dpad_up) {
                armPosition = ARM_ATTACH_HANGING_HOOK;

            } else if (gamepad2.dpad_left) {
                armPosition = ARM_WINCH_ROBOT;

            }

            /* Here we create a "fudge factor" for the arm position.
            This allows you to adjust (or "fudge") the arm position slightly with the gamepad triggers.
            We want the left trigger to move the arm up, and right trigger to move the arm down.
            So we add the right trigger's variable to the inverse of the left trigger. If you pull
            both triggers an equal amount, they cancel and leave the arm at zero. But if one is larger
            than the other, it "wins out". This variable is then multiplied by our FUDGE_FACTOR.
            The FUDGE_FACTOR is the number of degrees that we can adjust the arm by with this function. */

            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));


            /* Here we set the target position of our arm to match the variable that was selected
            by the driver.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
            armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));

            ((DcMotorEx) armMotor).setPower(0.1);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            viperMotor.setTargetPosition((int) (viperposition + viperPositionFudgeFactor));

            /* TECH TIP: Encoders, integers, and doubles
            Encoders report when the motor has moved a specified angle. They send out pulses which
            only occur at specific intervals (see our ARM_TICKS_PER_DEGREE). This means that the
            position our arm is currently at can be expressed as a whole number of encoder "ticks".
            The encoder will never report a partial number of ticks. So we can store the position in
            an integer (or int).
            A lot of the variables we use in FTC are doubles. These can capture fractions of whole
            numbers. Which is great when we want our arm to move to 122.5°, or we want to set our
            servo power to 0.5.

            setTargetPosition is expecting a number of encoder ticks to drive to. Since encoder
            ticks are always whole numbers, it expects an int. But we want to think about our
            arm position in degrees. And we'd like to be able to set it to fractions of a degree.
            So we make our arm positions Doubles. This allows us to precisely multiply together
            armPosition and our armPositionFudgeFactor. But once we're done multiplying these
            variables. We can decide which exact encoder tick we want our motor to go to. We do
            this by "typecasting" our double, into an int. This takes our fractional double and
            rounds it to the nearest whole number.
            */

            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) armMotor).isOverCurrent()) {
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }


            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("armTarget: ", armMotor.getTargetPosition());
            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
            telemetry.addData("Wrist Servo Direction",wrist.getDirection());
            telemetry.addData("wrist position",wrist.getPosition());
            telemetry.addData("Viper Position",viperMotor.getCurrentPosition());
            telemetry.update();

        }
    }
}
