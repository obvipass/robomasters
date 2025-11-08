package Vishruth.Libray.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Ensures the power stays within the min and max limits.
 */
public class Motor {
    private DcMotor motor;
    private double motorPowerMax;
    private double motorPowerMin;
    private double motorPower;

    public Motor(DcMotor motor, double motorPowerMax, double motorPowerMin) {
        this.motorPowerMax = Math.min(motorPowerMax, 1.0);
        this.motorPowerMin = Math.max(motorPowerMin, -1.0);
        this.motor = motor;
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    /**
     * Sets the motor power, between min and max.
     */
    public void setPower(double motorPower) {
        Range.clip(motorPower,motorPowerMin,motorPowerMax);
        this.motorPower = motorPower;
        motor.setPower(motorPower);
    }

    boolean isBusy = motor.isBusy();

    int getTargetPosition(){
        return motor.getTargetPosition();
    }
    int getCurrentPosition(){
        return motor.getCurrentPosition();
    }
    public void setMode(DcMotor.RunMode R){
        motor.setMode(R);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior z){
        motor.setZeroPowerBehavior(z);
    }

    public void setTargetPosition(int i){
        motor.setTargetPosition(i);
    }

    public void setDirection(DcMotorSimple.Direction d){
        motor.setDirection(d);
    }
    /**
     * Returns the current motor power.
     */
    public double getMotorPower() {
        return motorPower;
    }

}
