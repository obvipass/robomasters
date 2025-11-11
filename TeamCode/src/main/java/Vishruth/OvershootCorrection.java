package Vishruth;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Vishruth.Library.Library.SubAssemblies.MecanumDriveTrain;
@Autonomous
public class OvershootCorrection extends LinearOpMode {

    MecanumDriveTrain mecanumDriveTrain;

    @Override
    public void runOpMode() {
        mecanumDriveTrain = new MecanumDriveTrain(this);
        waitForStart();
        mecanumDriveTrain.moveInchesWithCOC(0.5,45,45);
    }
}
