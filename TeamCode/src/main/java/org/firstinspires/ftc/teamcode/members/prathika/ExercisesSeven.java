package org.firstinspires.ftc.teamcode.members.prathika;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.members.prathika.ProgrammingBoard4;

         @TeleOp()
 public class ExercisesSeven extends OpMode {
 ProgrammingBoard4 board = new ProgrammingBoard4();
 @Override
 public void init() {
         board.init(hardwareMap);
        }

         @Override
 public void loop() {
         if(gamepad1.a) {
             board.setMotorSpeed(0.5);
            }
         else{
             board.setMotorSpeed(0.0);
             }
         telemetry.addData("Motor rotations", board.getMotorRotations());
         }
 }
