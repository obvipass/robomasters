package org.firstinspires.ftc.teamcode.members.saksham.starterbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.FRLib.robot.StarterBot;
import org.firstinspires.ftc.teamcode.utils.Logger;

import java.util.LinkedHashMap;
import java.util.Map;

@TeleOp
public class DevCompTeleop extends LinearOpMode {
    private final Logger logger = new Logger(telemetry);
    private StarterBot robot;

    private double DRIVE_POWER = 0.35;

    private double TARGET_LAUNCHER_VELOCITY = 1500;
    private double MIN_LAUNCHER_VELOICTY = TARGET_LAUNCHER_VELOCITY - 20;
    private final Map<Double, String> velocityList = new LinkedHashMap<>(Map.of(
            1300.0, "mod 1: lands 2-4 ft",

            1600.0, "mod 2: lands 4-6 ft",
            1700.0, "mod 2: lands 4-6.5 ft",
            1750.0, "mod 2: lands 4.5 ft",

            1500.0, "mod 2.1: goal 5.5 ft",

            2000.0, "mod 3: lands 13-14 ft",
            2150.0, "mod 3: goal 13-14 ft"
    ));

    private final double FEED_TIME_SECONDS = 0.4;
    private final ElapsedTime feederTimer = new ElapsedTime();

    private enum LaunchState {
        IDLE, // balls loaded, waiting
        SPIN_UP, // spin up launch flywheel
        LAUNCH, // feed ball forward using servos
        LAUNCHING, // ball launched/launching, wait to turn off feeders
    }

    private LaunchState launchState = LaunchState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new StarterBot(this, logger);
        waitForStart();
        while (opModeIsActive()) {
            changeSpeed();
            arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);

            updateState();
            logger.logData(Logger.LoggerMode.CRITICAL, "launcherSpeed", robot.launcher.getVelocity());

            // press B to spin down flywheel
            if (gamepad1.bWasPressed()) {
                logger.tempLog(Logger.LoggerMode.CRITICAL, 7, "Spinning down flywheel");
                launchState = LaunchState.IDLE;
                robot.launcher.setVelocity(0);
            }

            if (gamepad1.dpadUpWasPressed()) TARGET_LAUNCHER_VELOCITY += 10;
            if (gamepad1.dpadDownWasPressed()) TARGET_LAUNCHER_VELOCITY -= 10;

            TARGET_LAUNCHER_VELOCITY = Math.min(TARGET_LAUNCHER_VELOCITY, robot.MAX_CAPABLE_VELOCITY);

            logger.logData(Logger.LoggerMode.CRITICAL, "Target Launcher Velocity", TARGET_LAUNCHER_VELOCITY);
            logger.logData(Logger.LoggerMode.CRITICAL, "Preset", velocityList.get(TARGET_LAUNCHER_VELOCITY));

            logger.update();
        }
    }

    private void updateState() {
        switch (launchState) {
            case IDLE:
                // normal launch
                if (gamepad1.rightBumperWasPressed()) {
                    launchState = LaunchState.SPIN_UP;
                } else {
                    logger.logData(Logger.LoggerMode.CRITICAL, "Feeder Status", "off (idle)");
                }
                break;
            case SPIN_UP:
                robot.launcher.setVelocity(TARGET_LAUNCHER_VELOCITY);
                if (robot.launcher.getVelocity() >= MIN_LAUNCHER_VELOICTY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                logger.logData(Logger.LoggerMode.CRITICAL, "Feeder Status", "forward (launch)");
                robot.leftFeeder.setPower(1);
                robot.rightFeeder.setPower(1);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    logger.logData(Logger.LoggerMode.CRITICAL, "Feeder Status", "stopped (done launching)");
                    robot.leftFeeder.setPower(0);
                    robot.rightFeeder.setPower(0);
                    launchState = LaunchState.IDLE; // idle moves servos backwards, 0 power above is a "precaution"
                }
                break;
        }

        logger.logData(Logger.LoggerMode.CRITICAL, "launchState", launchState);
    }

    private void changeSpeed() {
        if (gamepad1.xWasPressed()) DRIVE_POWER += 0.05;
        if (gamepad1.aWasPressed()) DRIVE_POWER -= 0.05;
        DRIVE_POWER = Range.clip(DRIVE_POWER, 0, 1);
        logger.logData(Logger.LoggerMode.CRITICAL, "Speed", DRIVE_POWER);
    }

    private void arcadeDrive(double axial, double yaw) {
        double leftPower = (axial + yaw);
        double rightPower = (axial - yaw);

        robot.leftDrive.setPower(leftPower * DRIVE_POWER);
        robot.rightDrive.setPower(rightPower * DRIVE_POWER);
    }
}
