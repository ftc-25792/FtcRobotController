package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
public class ColorSensor2 extends LinearOpMode {
    ColorSensor colorSensor;

    @Override
    public void runOpMode() {
       waitForStart();
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        while(opModeIsActive()) {
                if (colorSensor.red()> colorSensor.blue() && colorSensor.red()>colorSensor.green()) {
                    telemetry.addData("Color", "RedDetected");
                    telemetry.update();

                }

        }
        }


    }

