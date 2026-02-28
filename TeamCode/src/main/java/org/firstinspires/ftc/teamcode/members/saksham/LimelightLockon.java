package org.firstinspires.ftc.teamcode.members.saksham;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FRLib.robot.DecodeRobot;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.OurOpmode;

@Autonomous
public class LimelightLockon extends LinearOpMode {
    DecodeRobot robot;
    Logger logger;

    protected void initialize() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        logger = new Logger(telemetry);
        robot = new DecodeRobot(this, logger);
        robot.limelight.switchPipeline(0);
        logger.logData("Status", "Init'd");
        logger.update();

        waitForStart();


        while (opModeIsActive()) {
            if (robot.limelight.didPipelineSwitch()) {
                LLResult result = robot.limelight.getResultAndLog();
                if (result == null || !result.isValid()) {
                    robot.drive.stop();
                    continue;
                };

                double tx = result.getTx();
                robot.drive.driveVector(0, 0, tx / (robot.limelight.fovX / 2), 0.5);
            }

            logger.update();
        }
    }
}
