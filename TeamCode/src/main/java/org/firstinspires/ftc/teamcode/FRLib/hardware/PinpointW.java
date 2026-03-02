package org.firstinspires.ftc.teamcode.FRLib.hardware;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.Logger;

import java.util.Locale;

public class PinpointW {
    private GoBildaPinpointDriver pinpoint;
    private Logger logger;

    public PinpointW(HardwareMap hardwareMap, Logger logger) {
        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        this.logger = logger;
        this.pinpoint.setOffsets(5, -4, DistanceUnit.INCH);
        this.pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        this.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        this.pinpoint.resetPosAndIMU();
    }

    public Pose2D getPosition() {
        pinpoint.update();
        return this.pinpoint.getPosition();
    }

    public double getX() {
        return this.getPosition().getX(DistanceUnit.INCH);
    }

    public double getY() {
        return this.getPosition().getY(DistanceUnit.INCH);
    }

    public double getHeading() {
        return this.getPosition().getHeading(AngleUnit.DEGREES);
    }

    public void reset() {
        this.pinpoint.resetPosAndIMU();
    }

    public void logPositon() {
        Pose2D pos = this.getPosition();
        logger.logData("Position",
        String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES)));
    }
}
