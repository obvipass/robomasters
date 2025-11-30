package org.firstinspires.ftc.teamcode.members.prathika;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class crazyButton {
@Override

    public void loop(){

    double x = gamepad1.left_stick_x;
    double y = gamepad1.left_stick_y;

        if(gamepad1.left_stick_y){
            double temp = x;
            x = y;
            y = temp;
        }
    telemetry.addData("Crazy mode", gamepad1.a ? "Onn" : "off");
    telemetry.addData("X", x);
    telemetry.addData("Y", y);
    telemetry.update();
}
    }

