package Vishruth;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Autonomous
public class StatMachineTest extends LinearOpMode {
    MecanumDriveTrain mecanumDriveTrain;
    @Override
    public void runOpMode(){
        mecanumDriveTrain = new MecanumDriveTrain(hardwareMap,telemetry);
    }
}
