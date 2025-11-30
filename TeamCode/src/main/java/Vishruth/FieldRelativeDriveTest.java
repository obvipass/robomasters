package Vishruth;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Vishruth.Library.Library.SubAssemblies.MecanumDriveTrain;

@TeleOp
public class FieldRelativeDriveTest extends LinearOpMode {


    MecanumDriveTrain drive ;
    double axial;
    double lateral;
    double yaw;
    @Override
    public void runOpMode() {
       drive = new MecanumDriveTrain(this, MecanumDriveTrain.DriveTrainName.KEVIN);
       waitForStart();

       while (opModeIsActive()) {

           if (gamepad1.a){
               drive.imu.resetYaw();
           }

           axial = -gamepad1.left_stick_y;
           lateral = gamepad1.left_stick_x;
           yaw = gamepad1.right_stick_x;

           telemetry.addData("axial", axial);
           telemetry.addData("lateral", lateral);
           telemetry.addData("yaw", yaw);
           telemetry.update();

           drive.moveFieldRelative(axial, lateral, yaw);
       }

    }


}
