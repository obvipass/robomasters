package org.firstinspires.ftc.teamcode.members.vishruth;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FRLib.hardware.Distance2mW;
import org.firstinspires.ftc.teamcode.FRLib.hardware.IMUW;
import org.firstinspires.ftc.teamcode.FRLib.robot.Robot;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;

@Autonomous
public class DriveUntilSeeSomethingVishruth extends LinearOpMode {

    Logger logger = new Logger(telemetry);
    Robot robot;
    IMUW imu;
    Distance2mW distanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this,logger, MecanumDrive.RobotName.KEVIN);
        imu = new IMUW(hardwareMap,"imu", RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        distanceSensor = new Distance2mW(hardwareMap,"distance_sensor");
        logger.logData(Logger.LoggerMode.STATUS,"Hardware","Initialized");
        logger.update();
        waitForStart();
        robot.drive.driveStraightDistanceSensor(robot.imu,0,60,0.2f,100,robot.distanceSensor,10);
    }
}
