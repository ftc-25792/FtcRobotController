package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Custom Shooter Velocity PID", group="Linear Opmode")
public class CustomShooterPID extends LinearOpMode {

    DcMotorEx launcher;

    // PIDF values (start here)
    double P = 0 ; //0.0006;
    double I = 0.0;
    double D = 0 ;//0.00005;
    double F = 0.32;

    double targetVelocity = 1800;

    double lastError = 0;
    double integral = 0;

    boolean shooterOn = false;

    boolean lastA, lastUp, lastDown, lastLeft, lastRight, lastY, lastX;

    @Override
    public void runOpMode() {

        launcher = hardwareMap.get(DcMotorEx.class, "launcherLeft");

        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while (opModeIsActive()) {

            // ---------- Toggle shooter ----------
            if (gamepad1.a && !lastA) {
                shooterOn = !shooterOn;
            }

            // ---------- Tune P ----------
            if (gamepad1.dpad_up && !lastUp) P += 0.00005;
            if (gamepad1.dpad_down && !lastDown) P -= 0.00005;

            // ---------- Tune D ----------
            if (gamepad1.dpad_right && !lastRight) D += 0.00001;
            if (gamepad1.dpad_left && !lastLeft) D -= 0.00001;

            // ---------- Tune Target ----------
            if (gamepad1.y && !lastY) targetVelocity += 50;
            if (gamepad1.x && !lastX) targetVelocity -= 50;

            // Tune F
            if(gamepad1.right_bumper) F+= 0.01;
            if(gamepad1.left_bumper) F-=0.01;

            F= Math.max(0,Math.min(1.0, F));

            // Save button states
            lastA = gamepad1.a;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;
            lastY = gamepad1.y;
            lastX = gamepad1.x;

            double power = 0;

            if (shooterOn) {

                double currentVelocity = launcher.getVelocity();
                double error = targetVelocity - currentVelocity;

                // Integral (clamped)
                integral += error;
                integral = Math.max(-2000, Math.min(2000, integral));

                // Derivative
                double derivative = error - lastError;

                // PIDF calculation
                power = (P * error)
                        + (I * integral)
                        + (D * derivative)
                        + F;

                // Clip power
                power = Math.max(0, Math.min(1.0, power));

                launcher.setPower(power);

                lastError = error;

            } else {

                launcher.setPower(0);
                integral = 0;
                lastError = 0;
            }

            // ---------- Telemetry ----------
            telemetry.addLine("=== CUSTOM SHOOTER PID ===");

            telemetry.addData("Shooter On", shooterOn);
            telemetry.addData("Target Vel", targetVelocity);
            telemetry.addData("Current Vel", launcher.getVelocity());
            telemetry.addData("Power", launcher.getPower());

            telemetry.addLine("----------------");

            telemetry.addData("P ","%.5f" ,P);
            telemetry.addData("I", I);
            telemetry.addData("D", "%.5f", D);
            telemetry.addData("F", "%.5f", F);

            telemetry.addLine("----------------");

            telemetry.addLine("A = Toggle Shooter");
            telemetry.addLine("Dpad Up/Down = P");
            telemetry.addLine("Dpad L/R = D");
            telemetry.addLine("Y/X = Target");

            telemetry.update();

            sleep(30);
        }
    }
}
