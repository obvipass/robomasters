package org.firstinspires.ftc.teamcode.members.vishruth;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.FRLib.hardware.IMUW;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.VelocityProfilerDynamic;

@TeleOp
public class MecanumLauncherV2 extends OurOpmode{
    Launcher launcher;
    MecanumDrive drive;
    IMUW imu;
    double power;
    VelocityProfilerDynamic profiler = new VelocityProfilerDynamic(0.075);
    double counter = 0;

    enum DriverPreference {
        FIELD_RELATIVE,
        NORMAL
    }

    DriverPreference preference = DriverPreference.NORMAL;
    @Override
    protected void Loop() {
        power = gamepad1.right_bumper ? 0.82 : 0.5;

        if (gamepad1.leftBumperWasPressed()) {
            preference = (preference == DriverPreference.NORMAL)
                    ? DriverPreference.FIELD_RELATIVE
                    : DriverPreference.NORMAL;
        }


        switch  (preference) {
            case FIELD_RELATIVE:  drive.driveVectorField(
                    profiler.velocityProfile("Left Stick Y", -gamepad1.left_stick_y),
                    profiler.velocityProfile("Left Stick X", gamepad1.left_stick_x),
                    profiler.velocityProfile("Right Stick X", -gamepad1.right_stick_x),
                    power, imu
            ); break;


            case NORMAL : drive.driveVector(
                    profiler.velocityProfile("Left Stick Y", -gamepad1.left_stick_y),
                    profiler.velocityProfile("Left Stick X", gamepad1.left_stick_x),
                    profiler.velocityProfile("Right Stick X", -gamepad1.right_stick_x),
                    power); break;
        }


        launcher.launchTeleOp(gamepad2.rightBumperWasPressed());
        
        if (gamepad2.bWasPressed()){
            launcher.stopFlywheel();
            logger.logData(Logger.LoggerMode.CRITICAL,"Flywheel","Stopped");
        }

        if (gamepad1.yWasPressed()){
            imu.resetYaw();
            logger.logData("imu","reset");
        } else {
            logger.logData("imu","Not Reset");
        }
//        if(gamepad1.xWasPressed()){
//            power = power + 0.1;
//        } else if (gamepad1.aWasPressed()) {
//            power = power -0.1;
//        }
        if (gamepad2.yWasPressed()){
            launcher.spinUpFlywheel();
            logger.logData(Logger.LoggerMode.CRITICAL,"Flywheel","Spinning Up");
        }

        logVariables();
    }



    private void logVariables() {
        logger.logData(Logger.LoggerMode.STATUS,"FlyWheelVelocity",launcher.getLauncher().getVelocity());
        logger.logData("Driver Preference",preference);
        logger.logData("TargetVelocity",launcher.getLauncherTargetVelocity());
        logger.logData(Logger.LoggerMode.STATUS,"Launch-state",launcher.getLaunchStatesT());
        logger.logData("FrontLeft",drive.frontLeft.getRawMotor().getPower());
        logger.logData("FrontRight",drive.frontRight.getRawMotor().getPower());
        logger.logData("RearLeft",drive.rearLeft.getRawMotor().getPower());
        logger.logData("RearRight",drive.rearRight.getRawMotor().getPower());
        logger.update();

    }

    @Override
    protected void initialize() {
        profiler.addInput("Left Stick Y");
        profiler.addInput("Left Stick X");
        profiler.addInput("Right Stick X");
        logger = new Logger(telemetry);
        launcher = new Launcher(this);
        drive = new MecanumDrive(this,logger, MecanumDrive.RobotName.BOB);
        imu = new IMUW(hardwareMap,"imu", RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        drive.setMotorZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.BRAKE);
        imu.resetYaw();
        logger.logData(Logger.LoggerMode.CRITICAL,"Status","Init");
        logger.update();
    }
}
