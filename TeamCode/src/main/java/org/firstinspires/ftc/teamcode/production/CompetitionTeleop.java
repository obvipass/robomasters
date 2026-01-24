package org.firstinspires.ftc.teamcode.production;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.FRLib.robot.DecodeRobot;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.VelocityProfilerDynamic;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.OurOpmode;
@TeleOp
public class CompetitionTeleop extends OurOpmode {
    // init variables
    DecodeRobot bot;
    double power;
    VelocityProfilerDynamic profiler = new VelocityProfilerDynamic(0.1);
    // made 2 different possibilities for what the driver wants, Field Relative Or Without Field Relative, which is normal.

    enum DriverPreference {
        FIELD_RELATIVE,
        NORMAL
    }
    double fastPower = 0.82;
    double normalPower = 0.5;

    DriverPreference preference = DriverPreference.FIELD_RELATIVE;
    @Override
    protected void Loop() {
        // this if the leftBumper is constantly pressed, then do the power 0.82, otherwise, 0.5
        power = gamepad1.left_bumper ? fastPower : normalPower;
        // if the leftBumper was pressed, it checks what state the driver preference is and converts it to the other one



        driveBasedOnPreferences();


        bot.launcher.launchTeleOp(gamepad1.rightBumperWasPressed());

        if (gamepad1.bWasPressed()){
            bot.launcher.stopFlywheel();
            logger.logData(Logger.LoggerMode.CRITICAL,"Flywheel","Stopped");
        }

        if (gamepad2.yWasPressed()){
            bot.imu.resetYaw();
        }

        if (gamepad1.yWasPressed()){
            bot.launcher.spinUpFlywheel();
            logger.logData(Logger.LoggerMode.CRITICAL,"Flywheel","Spinning Up");
        }

        logVariables();
    }

    private void driveBasedOnPreferences() {
        // this says if the preference is field relative, the drive field relative, if the preference is normal, drive normal
        switch  (preference) {
            case FIELD_RELATIVE:  bot.drive.driveVectorField(
                    profiler.velocityProfile("Left Stick Y", -gamepad1.left_stick_y),
                    profiler.velocityProfile("Left Stick X", gamepad1.left_stick_x),
                    profiler.velocityProfile("Right Stick X", -gamepad1.right_stick_x),
                    power, bot.imu
            ); break;


            case NORMAL : bot.drive.driveVector(
                    profiler.velocityProfile("Left Stick Y", -gamepad1.left_stick_y),
                    profiler.velocityProfile("Left Stick X", gamepad1.left_stick_x),
                    profiler.velocityProfile("Right Stick X", -gamepad1.right_stick_x),
                    power); break;
        }
    }

    private void logInstructions(){
        logger.permLog(Logger.LoggerMode.STATUS,"Press Gamepad 1 LeftBumper for increased speed");
        logger.permLog(Logger.LoggerMode.STATUS,"Press Gamepad 1 B to stop flywheel");
        logger.permLog(Logger.LoggerMode.STATUS,"Press Gamepad 1 RightBumper to shoot");
        logger.permLog(Logger.LoggerMode.STATUS,"Press Gamepad 1 Y to start flywheel");
        logger.permLog(Logger.LoggerMode.STATUS,"Press Gamepad 2 Y to reset yaw");
        logger.update();
    }

    private void logVariables() {
        // logs all important information the driver needs
        logger.logData(Logger.LoggerMode.STATUS,"FlyWheelVelocity",bot.launcher.getLauncher().getVelocity());
        logger.logData("TargetVelocity",bot.launcher.getLauncherTargetVelocity());
        logger.logData("Driver Preference",preference);
        logger.logData("IMU Yaw", bot.imu.getYaw());
        logger.logData(Logger.LoggerMode.STATUS,"Launch-state",bot.launcher.getLaunchStatesT());
        logger.update();

    }

    @Override
    protected void initialize() {
        // profiler.addInput means when you call the velocity-profiling method, these are the appropriate names for the inputs that the profiler will recognise
        profiler.addInput("Left Stick Y");
        profiler.addInput("Left Stick X");
        profiler.addInput("Right Stick X");
        //initializing everything else
        logger = new Logger(telemetry);
        bot = new DecodeRobot(this,logger);
        bot.imu.resetYaw();
        bot.drive.setMotorZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.BRAKE);
        logger.logData(Logger.LoggerMode.CRITICAL,"Status","Init");
        logInstructions();
    }
}
