package org.firstinspires.ftc.teamcode.production;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FRLib.robot.DecodeRobot;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;

@Autonomous
public class CompetitionAuto extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();

    boolean wait = true;
    Logger logger = new Logger(telemetry);
    DecodeRobot bot;

    enum RobotPosition {
        NEAR,
        FAR;

        RobotPosition next() {
            return this == NEAR ? FAR : NEAR;
        }
    }

    RobotPosition position = RobotPosition.NEAR;

    @Override
    public void runOpMode() {

        while (opModeInInit()) {

            if (gamepad1.bWasPressed()) {
                position = position.next();
                logger.logData("Robot Position", position);
                logger.update();
            }

            if (gamepad1.aWasPressed()) {
                wait = !wait;
                logger.logData("Wait Or Not?", wait);
                logger.update();
            }
        }

        bot = new DecodeRobot(this, logger);

        waitForStart();
        timer.reset();

        while (timer.seconds() < 20 && wait) {
            idle();
        }

        switch (position) {
            case NEAR:
                bot.drive.driveStraight(0,24,0.6f);
                break;

            case FAR:
                bot.drive.driveStraight(0,100,0.6f);
                break;
        }
    }
}
