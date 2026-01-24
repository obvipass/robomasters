package org.firstinspires.ftc.teamcode.production;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FRLib.robot.DecodeRobot;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;

@Autonomous
public class CompetitionAuto extends LinearOpMode {
    Logger logger = new Logger(telemetry);
    DecodeRobot bot;
    enum RobotPositions {
        RED_CLOSE,
        RED_FAR,
        BLUE_FAR,
        BLUE_CLOSE;

        RobotPositions next() {
            switch (this) {
                case RED_CLOSE:  return RED_FAR;
                case RED_FAR:    return BLUE_FAR;
                case BLUE_FAR:   return BLUE_CLOSE;
                case BLUE_CLOSE: return RED_CLOSE;
                default: return null;
            }
        }
    }
    RobotPositions position = RobotPositions.BLUE_CLOSE;


    @Override
    public void runOpMode() {
        while (opModeInInit()) {
            if(gamepad1.bWasPressed()){
                position = position.next();
            }
            logger.logData("Robot Position",position);
        }
        bot = new DecodeRobot(this,logger);
        waitForStart();
        switch (position){
            case RED_CLOSE:
                bot.drive.driveDistance(MecanumDrive.Direction.FORWARD,24,0.6,false);
                bot.drive.driveDistance(MecanumDrive.Direction.LEFT,12,0.6,false);
                break;
            case BLUE_CLOSE:
                bot.drive.driveDistance(MecanumDrive.Direction.FORWARD,24,0.6,false);
                bot.drive.driveDistance(MecanumDrive.Direction.RIGHT,12,0.6,false);
                break;
            case RED_FAR:
                bot.drive.driveDistance(MecanumDrive.Direction.BACKWARD,50,0.6,false);
                bot.drive.driveDistance(MecanumDrive.Direction.LEFT,36,0.6,false);
                break;
            case BLUE_FAR:
                bot.drive.driveDistance(MecanumDrive.Direction.BACKWARD,50,0.6,false);
                bot.drive.driveDistance(MecanumDrive.Direction.RIGHT,36,0.6,false);
                break;
        }

    }
}
