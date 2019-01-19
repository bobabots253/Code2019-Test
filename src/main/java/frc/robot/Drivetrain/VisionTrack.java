package frc.robot.Drivetrain;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.OI.PIDController;

public class VisionTrack extends Command {

    private double left, right;

    private double aim_kP = 0.01;
    private double aim_kI = 0;
    private double aim_kD = 0.001;
    private double aim_kF = 0.05;

    private double dist_kP = 0.15;
    private double dist_kI = 0;
    private double dist_kD = 0;
    private double dist_kF = 0;

    private PIDController aim = new PIDController(aim_kP, aim_kI, aim_kD, 0.02);
    private PIDController distance = new PIDController(dist_kP, dist_kI, dist_kD, 0.02);

    public VisionTrack() {
        requires(Robot.drivetrain);
        /*
        aim.configure(0);
        distance.configure(0);*/
    }

    protected void execute() {

        double qLeft = Robot.oi.getLeftTrigger();
        double qRight = Robot.oi.getRightTrigger();

        if (!Robot.oi.getTargetValid()) {

            SmartDashboard.putBoolean("Has Target", false);

            

            DrivetrainSubsystem.drive(Robot.oi.throttleValue() + 0.5*(-qLeft+qRight) + 0.2, Robot.oi.throttleValue() + 0.5*(-qRight+qLeft) - 0.2);
            
        } else {

            SmartDashboard.putBoolean("Has Target", true);

            double heading_error = Robot.oi.getxOffset();
            double distance_error = Robot.oi.getyOffset();

            SmartDashboard.putNumber("heading_error", heading_error);

            double steering_adjust = aim.calculate(heading_error);
            double distance_adjust = distance.calculate(distance_error);

            left = distance_adjust - steering_adjust;
            right = distance_adjust + steering_adjust;

            left += left > 0 ? aim_kF : -aim_kF;
            right += right > 0 ? aim_kF : -aim_kF;

            DrivetrainSubsystem.drive(Robot.oi.throttleValue() + 0.5*(-qLeft+qRight) + left, Robot.oi.throttleValue() + 0.5*(-qRight+qLeft) + right);

            SmartDashboard.putNumber("left", left);
            SmartDashboard.putNumber("right", right);

        }

    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
        DrivetrainSubsystem.drive(0, 0);
    }

}