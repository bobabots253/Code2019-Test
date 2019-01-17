package frc.robot.OI;

public class PIDController {

    private double kP, kI, kD, dt;
    private double last_error = 0;

    public PIDController(double kP, double kI, double kD, double dt){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.dt = dt;
    }

    public void configure(double initial_error){
        this.last_error = initial_error;
    }

    public double calculate(double error){
        double ret = kP * error + kD * (error - last_error)/dt;
        this.last_error = error;

        return ret;
    }
}