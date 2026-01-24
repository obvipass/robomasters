package org.firstinspires.ftc.teamcode.members.vishruth;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
// @TeleOp
public class ColorSensorVishruth extends LinearOpMode {
    NormalizedColorSensor colorSensor;
    Color colors;
    @Override
    public void runOpMode() throws InterruptedException {
        colors = new Color();

        float gain = 170;

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor.setGain(gain);
        telemetry.addLine("Color Sensor Init");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){



            if (gamepad1.a) {
                gain +=10;
            } else
            if (gamepad1.b) {
                gain -= 10;
            }

            if (gamepad1.x){
                telemetry.addData("red",colorSensor.getNormalizedColors().red);
                telemetry.addData("green",colorSensor.getNormalizedColors().green);
                telemetry.addData("blue",colorSensor.getNormalizedColors().blue);
                telemetry.addData("gain",gain);
                telemetry.update();
            }
        }
    }
}
