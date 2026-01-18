package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FRLib.robot.Robot;

public abstract class  OurOpmode extends LinearOpMode {
    protected Robot robot;
    protected Logger logger;
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
