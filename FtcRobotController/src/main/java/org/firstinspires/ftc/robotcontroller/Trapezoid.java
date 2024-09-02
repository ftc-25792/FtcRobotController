package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Robot: Auto Square", group="Autonomous")
public class Trapezoid extends LinearOpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    double speed = 0.75;
    long moveDuration = 700; // Move duration in milliseconds. Change this number to adjust the distance traveled per side.
    long turnDuration = 325; // Turn duration in milliseconds

    @Override
    public void runOpMode() {
        // Initialize the hardware variables.
        frontLeft = hardwareMap.get(DcMotor.class, "Left_front");
        frontRight = hardwareMap.get(DcMotor.class, "Right_front");
        backLeft = hardwareMap.get(DcMotor.class, "Left_rear");
        backRight = hardwareMap.get(DcMotor.class, "Right_rear");
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {
            // Move forward
            move(speed, speed, speed, speed, moveDuration);
            // Stop moving forward
            move(0, 0, 0, 0, 500);

            // R-Back strafe 45 degrees
            move(-speed, 0, 0, -speed, turnDuration);
            // Stop strafing
            move(0, 0, 0, 0, 500);

            // Move backward
            move(-speed, -speed, -speed, -speed, 550);
            // Stop moving backward
            move(0, 0, 0, 0, 500);

            // L-Back strafe 45 degrees
            move(0, -speed, -speed, 0, turnDuration);
            // Stop strafing
            move(0, 0, 0, 0, 500);
        }
    }

    // Updated move method definition
    public void move(double a, double b, double c, double d, long e) {
        frontLeft.setPower(a);
        frontRight.setPower(b);
        backLeft.setPower(c);
        backRight.setPower(d);
        sleep(e);
    }
}
