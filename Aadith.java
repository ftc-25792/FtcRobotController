package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Robot: Auto Rectangle", group="Autonomous")
public class RectangleAutoRun extends LinearOpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    double speed = 0.75;

    long longSideDuration = 1000;  //(36 inches)
    long shortSideDuration = 700;  // (24 inches)
    long turnDuration = 325;
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
            for (int i = 0; i < 2; i++) {
                // Move along the longer side
                frontLeft.setPower(speed);
                frontRight.setPower(speed);
                backLeft.setPower(speed);
                backRight.setPower(speed);
                sleep(longSideDuration);

                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                sleep(500); // Short pause before turning

                frontLeft.setPower(speed);
                frontRight.setPower(-speed);
                backLeft.setPower(speed);
                backRight.setPower(-speed);
                sleep(turnDuration);


                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                sleep(500); // Short pause

                // Move along the shorter side
                frontLeft.setPower(speed);
                frontRight.setPower(speed);
                backLeft.setPower(speed);
                backRight.setPower(speed);
                sleep(shortSideDuration);

                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                sleep(500); // Short pause before turning


                frontLeft.setPower(speed);
                frontRight.setPower(-speed);
                backLeft.setPower(speed);
                backRight.setPower(-speed);
                sleep(turnDuration);


                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                sleep(500); // Short pause
            }
        }
    }
}
