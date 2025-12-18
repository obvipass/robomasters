package org.firstinspires.ftc.teamcode.members.saksham.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FRLib.robot.Robot;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.slf4j.LoggerFactory;

@Autonomous
public class ObstacleAvoidance_Saksham extends LinearOpMode {
    Logger logger = new Logger(telemetry);
    double distanceTraveled = 0;
    double targetInches = 84;
    final double sidePass = 20;
    final double forwardPass = 40;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, logger, MecanumDrive.RobotName.KEVIN);
        waitForStart();

        if (opModeIsActive()) {
            distanceTraveled += robot.drive.driveStraightUntilObstacle(targetInches, 10, robot.distanceSensor, 0.2);
            logger.logData(Logger.LoggerMode.CRITICAL, "distanceTraveled", distanceTraveled);
            logger.update();
            robot.drive.driveDistance(MecanumDrive.Direction.RIGHT, sidePass, 0.2, true);

            robot.drive.driveStraight((float) (0 - robot.imu.getYaw()), (float)forwardPass, 0.2f);
            distanceTraveled += forwardPass;
            logger.logData(Logger.LoggerMode.CRITICAL, "distanceTraveled", distanceTraveled);
            logger.update();

            robot.drive.driveDistance(MecanumDrive.Direction.LEFT, sidePass, 0.2, true);

            logger.logData(Logger.LoggerMode.CRITICAL, "remaining", targetInches - distanceTraveled);
            logger.update();
            robot.drive.driveStraight((float) (0 - robot.imu.getYaw()), (float) (targetInches - distanceTraveled), 0.2f);
        }
    }
}
