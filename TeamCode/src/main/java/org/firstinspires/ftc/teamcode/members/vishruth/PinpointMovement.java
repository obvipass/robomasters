package org.firstinspires.ftc.teamcode.members.vishruth;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.FRLib.hardware.IMUW;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.OurOpmode;
@Autonomous
public class PinpointMovement extends OurOpmode {
    Logger log;
    MecanumDrive drive;
    IMUW imu;
    GoBildaPinpointDriver pinpoint;
    Pose2D pose1 = new Pose2D(DistanceUnit.INCH,60,-24, AngleUnit.DEGREES,90);
    Pose2D pose2 = new Pose2D(DistanceUnit.INCH,90,60,AngleUnit.DEGREES,0);

    enum States {
        IDLE,
        PATH1,
        TURNING,
        PATH2
    }
    States states = States.IDLE;
    Pose2D currentPose;
    @Override
    protected void Loop() {

        telemetry.addData("X coordinate (IN)", currentPose.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", currentPose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", currentPose.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    public void updateStates() {
        switch (states){
            case IDLE: if(this.opModeIsActive()){
                states = States.PATH1;
            } break;
            case PATH1: pinpoint.update();
                drive.driveToPose(pose1,pinpoint.getPosition(),0.7);
                currentPose=pinpoint.getPosition();
                if(!drive.isAnyMotorBusy()){
                    states = States.TURNING;
                }
                break;
            case TURNING: pinpoint.update();
                drive.turnDegrees(90,0.5,0.2,pinpoint.getPosition().getHeading(AngleUnit.DEGREES));
                if(!drive.isAnyMotorBusy()){
                    states = States.PATH2;
                }
        }
    }


    @Override
    protected void initialize() {

        imu = new IMUW(hardwareMap,"imu", RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        log = new Logger(this.telemetry);
        drive = new MecanumDrive(this,log, MecanumDrive.RobotName.BOB,imu);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.DEGREES,0));
    }

    public void configurePinpoint(){
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(5, -4, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }
}
