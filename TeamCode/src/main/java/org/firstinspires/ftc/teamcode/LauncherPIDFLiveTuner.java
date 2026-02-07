package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name = "Launcher PIDF Live Tuner", group = "Linear Opmode")
public class LauncherPIDFLiveTuner extends LinearOpMode {

    public static final double P_INC_DEC = 0.00005;
    public static final double D_INC_DEC = 0.00001;
    public static final double F_INC_DEC = 0.00001;
    DcMotorEx launcher;

    // Starting values (safe for goBILDA 5202)
    double P = 0.0006;
    double I = 0.0;
    double D = 0.00006;
    double F_start =14;
    double F = F_start ;
    double F_final = 1;

    double target = 1800;

    // Button state tracking
    boolean lastUp, lastDown, lastLeft, lastRight;
    boolean lastA, lastB, lastX, lastY;

    @Override
    public void runOpMode() {

        launcher = hardwareMap.get(DcMotorEx.class, "launcherRight");

        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        waitForStart();
        while (launcher.getVelocity() < 50 && opModeIsActive()) {
            launcher.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            launcher.setPower(0.5);
            telemetry.addLine("Trying to start");
            telemetry.update();
            sleep(500);


        }
        launcher.setPower(0);
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
// LIVE
        while (opModeIsActive()) {


            // ---------------------------
            // Adjust PIDF with D-Pad
            // ---------------------------

            // P : Up / Down
            if (gamepad1.dpad_up && !lastUp) P += P_INC_DEC;
            if (gamepad1.dpad_down && !lastDown) P -= P_INC_DEC;

            // D : Left / Right
            if (gamepad1.dpad_left && !lastLeft) D -= D_INC_DEC;
            if (gamepad1.dpad_right && !lastRight) D += D_INC_DEC;

            // F : X / B
            if (gamepad1.x && !lastX) F -= F_INC_DEC;
            if (gamepad1.b && !lastB) F += F_INC_DEC;

            // Target speed : Y / A
            if (gamepad1.y && !lastY) target += 50;
            if (gamepad1.a && !lastA) target -= 50;

            // Save button states
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;
            lastX = gamepad1.x;
            lastB = gamepad1.b;
            lastY = gamepad1.y;
            lastA = gamepad1.a;

            double rampStep = 1;
            if(F > F_final)
            {
                F-= rampStep;
                if(F<F_final) F = F_final;
            }

            // ---------------------------
            // Apply PIDF
            // ---------------------------
            launcher.setVelocityPIDFCoefficients(P, I, D, F);
            launcher.setVelocity(target);

                       // ---------------------------
            // Telemetry
            // ---------------------------

            double vel = launcher.getVelocity();
            double error = target - vel;

            telemetry.addLine("=== PIDF LIVE TUNER ===");
            telemetry.addLine("Hold RIGHT BUMPER = Spin Shooter");

            telemetry.addData("Target", target);
            telemetry.addData("Velocity", vel);
            telemetry.addData("Error", error);
            telemetry.addData("Power", launcher.getPower());

            telemetry.addLine("--------------------");

            telemetry.addData("P (Up/Down)", P);
            telemetry.addData("D (Left/Right)", D);
            telemetry.addData("F (B+/X-)", F);

            telemetry.addLine("--------------------");

            telemetry.addLine("Y/A = Target +/-");
            telemetry.addLine("X/B = F -/+");
            telemetry.addLine("Dpad = P/D");


            telemetry.update();

            sleep(5000); // prevents button spam
        }

    }
}