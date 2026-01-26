package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Drake<Kendrick", group="Autonomous")
public class CertifiedLoverBoy extends LinearOpMode {

    private DcMotorEx launcherLeft, launcherRight;

    // Motor / encoder specs
    private static final int ENCODER_TICKS_PER_REV = 28; // RS-555 output shaft
    private static final double MAX_RPM = 6000;
    private static final double MAX_TPS = MAX_RPM * ENCODER_TICKS_PER_REV / 60.0;

    private static final double TARGET_SPEED_PERCENT = 0.7; // 70% of max speed
    private static final double TARGET_TPS = MAX_TPS * TARGET_SPEED_PERCENT;

    private static final double Kp = 0.0005;
    private static final double Ki = 0.000001;
    private static final double Kd = 0.0001;

    @Override
    public void runOpMode() {

        // Hardware mapping
        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");

        // Motor directions
        launcherLeft.setDirection(DcMotorEx.Direction.REVERSE);
        launcherRight.setDirection(DcMotorEx.Direction.FORWARD);

        telemetry.addLine("Launcher PID Initialized");
        telemetry.addData("Target TPS (70%)", TARGET_TPS);
        telemetry.update();

        waitForStart();

        double integralL = 0, integralR = 0;
        double lastErrorL = 0, lastErrorR = 0;
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {

            double dt = timer.seconds();
            timer.reset();

            double velL = launcherLeft.getVelocity();
            double velR = launcherRight.getVelocity();

            double errorL = TARGET_TPS - velL;
            integralL += errorL * dt;
            double derivativeL = (errorL - lastErrorL) / dt;
            double powerL = Kp * errorL + Ki * integralL + Kd * derivativeL;
            powerL = Range.clip(powerL, 0, 1);
            lastErrorL = errorL;

            double errorR = TARGET_TPS - velR;
            integralR += errorR * dt;
            double derivativeR = (errorR - lastErrorR) / dt;
            double powerR = Kp * errorR + Ki * integralR + Kd * derivativeR;
            powerR = Range.clip(powerR, 0, 1);
            lastErrorR = errorR;


            launcherLeft.setPower(powerL);
            launcherRight.setPower(powerR);


            telemetry.addData("Launcher Left TPS", "%.1f TPS", velL);
            telemetry.addData("Launcher Right TPS", "%.1f TPS", velR);
            telemetry.addData("Power Left", "%.2f", powerL);
            telemetry.addData("Power Right", "%.2f", powerR);
            telemetry.update();

            sleep(20);
        }


        launcherLeft.setPower(0);
        launcherRight.setPower(0);
    }
}
