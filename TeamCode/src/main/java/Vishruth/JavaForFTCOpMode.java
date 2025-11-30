package Vishruth;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Vishruth.Library.Library.SubAssemblies.MecanumDriveTrain;
@TeleOp
public class JavaForFTCOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {

        waitForStart();

        while(opModeIsActive()){
            chapter3Exercises();
            chaper4exercises();
        }

    }


    public void chapter3Exercises(){
        telemetry.addData("Gamepad 1 RightStick",gamepad1.right_stick_button);
        telemetry.addData("Gamepad 1 B Button", gamepad1.b);
        telemetry.addData("Difference",gamepad1.left_stick_y-gamepad1.right_stick_y);
        telemetry.addData("sum",gamepad1.left_trigger+gamepad1.right_trigger);
        telemetry.update();
    }

    public void chaper4exercises(){
        if(gamepad1.a){
        telemetry.addData("Turbo","Active");
        telemetry.addData("Forward Speed",gamepad1.left_stick_y);
        telemetry.update();
        } else {
            telemetry.addData("Turbo","Inactive");
            telemetry.addData("Forward Speed",gamepad1.left_stick_y*0.5);
            telemetry.update();
        }

        if(gamepad1.b){
            telemetry.addData("Crazy Mode","Active");
            telemetry.addData("X",gamepad1.left_stick_y);
            telemetry.addData("Y",gamepad1.left_stick_x);
            telemetry.update();
        } else {
            telemetry.addData("Crazy Mode","Inactive");
            telemetry.addData("X",gamepad1.left_stick_x);
            telemetry.addData("Y",gamepad1.left_stick_y);
            telemetry.update();
        }


    }




}
