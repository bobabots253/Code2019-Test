package frc.robot.Drivetrain;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.OI.PIDController;

public class VisionTrack extends Command {

    
    private double left = 0, right = 0;
    
    private double aim_kP = 0.02;
    private double aim_kI = 0;
    private double aim_kD = 0;
    private double aim_kF = 0.09;

    private double dist_kP = 0;
    private double dist_kI = 0;
    private double dist_kD = 0;
    private double dist_kF = 0;

    private PIDController aim =  new PIDController(aim_kP, aim_kI, aim_kD, 0.02);
    private PIDController distance = new PIDController(dist_kP, dist_kI, dist_kD, 0.02);
    
    public VisionTrack() {
        requires(Robot.drivetrain);

        aim.configure(0);
        distance.configure(0);
    }

    protected void execute() {

        double heading_error = Robot.oi.getxOffset(); //when negative u want go right fast
        double distance_error = Robot.oi.getyOffset();

        SmartDashboard.putNumber("heading_error", heading_error);

        double steering_adjust = aim.calculate(heading_error);
        double distance_adjust = distance.calculate(distance_error);

        left = 0-steering_adjust;
        right = 0+steering_adjust;

        
        left += left > 0 ? aim_kF : -aim_kF;
        right += right > 0 ? aim_kF : -aim_kF;
        

        DrivetrainSubsystem.drive(left, right);
        SmartDashboard.putNumber("left", left);
        SmartDashboard.putNumber("right", right);


        System.out.println("applied" + left + "," + right);

    }

    protected boolean isFinished(){
        return false;
    }

    protected void end(){
        DrivetrainSubsystem.drive(0, 0);
    }



}