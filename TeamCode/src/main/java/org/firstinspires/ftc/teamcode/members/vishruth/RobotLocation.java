package org.firstinspires.ftc.teamcode.members.vishruth;

public class RobotLocation {
    double angle;

    public RobotLocation(double angle){
        if(angle>180){
            this.angle = angle-180;
        } else if (angle<180) {
            this.angle = angle + 180;
        }
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    @Override
    public String toString() {
        return "RobotLocation{" +
                "angle=" + angle +
                '}';
    }
}
