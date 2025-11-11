package Vishruth.Library.Library.Hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Imu {
    IMU imu;

    public void initIMU(HardwareMap map){
            imu = map.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public double getHeading(AngleUnit angleUnit){
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    public void resetYaw(){
        imu.resetYaw();

    }

}
