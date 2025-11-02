package Vishruth;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Ensures the power stays within the min and max limits.
 */
public class MotorPower {
    private DcMotor motor;
    private double motorPowerMax;
    private double motorPowerMin;
    private double motorPower;

    public MotorPower(DcMotor motor,double motorPowerMax, double motorPowerMin) {
        this.motorPowerMax = Math.min(motorPowerMax, 1.0);
        this.motorPowerMin = Math.max(motorPowerMin, -1.0);
        this.motor = motor;
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    /**
     * Sets the motor power, between min and max.
     */
    public void setMotorPower(double motorPower) {
        Range.clip(motorPower,motorPowerMin,motorPowerMax);
        this.motorPower = motorPower;
        motor.setPower(motorPower);
    }

    /**
     * Returns the current motor power.
     */
    public double getMotorPower() {
        return motorPower;
    }

}
