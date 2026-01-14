package org.firstinspires.ftc.teamcode.members.vishruth;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FRLib.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.Logger;

public abstract class  OurOpmode extends LinearOpMode {
    Robot robot;
    Logger logger;
    @Override
    public final void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (opModeIsActive()){
            Loop();
            idle();
        }
    }

    protected abstract void Loop();

    protected abstract void initialize();


}
