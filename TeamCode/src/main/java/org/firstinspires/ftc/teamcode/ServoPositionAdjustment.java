package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoPositionAdjust", group = "Testing")
public class ServoPositionAdjustment extends LinearOpMode {

    private Servo flapperLeft, flapperRight;
    private double flapperLeftPosition = 0.5;
    private double flapperRightPosition = 0.5;

    @Override
    public void runOpMode() {
        flapperLeft = hardwareMap.get(Servo.class, "fl");
        flapperRight = hardwareMap.get(Servo.class, "fr");

        flapperLeft.setPosition(flapperLeftPosition);
        flapperRight.setPosition(flapperRightPosition);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a){
                flapperLeftPosition += 0.01;
            }
            else if (gamepad1.b) {
                flapperLeftPosition -= 0.01;
            }
            else if (gamepad1.x) {
                flapperRightPosition += 0.01;
            }
            else if (gamepad1.y) {
                flapperRightPosition -= 0.01;
            }

            flapperLeftPosition = Math.max(0.0, Math.min(1.0, flapperLeftPosition));
            flapperRightPosition = Math.max(0.0, Math.min(1.0, flapperRightPosition));

            flapperLeft.setPosition(flapperLeftPosition);
            flapperRight.setPosition(flapperRightPosition);

            telemetry.addData("Flapper Left Pos","%.2f", flapperLeftPosition);
            telemetry.addData("Flapper Right Pos", "%.2f",flapperRightPosition);
            telemetry.update();
        }
    }
}
