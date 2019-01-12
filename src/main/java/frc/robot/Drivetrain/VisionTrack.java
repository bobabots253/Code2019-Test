package frc.robot.Drivetrain;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class VisionTrack extends Command {

    
    private double last_heading_error = 0, kDAim = 0.005;
    private double kPAim = -0.001, kPDistance = 0, min_aim_command = 0.13, left = 0, right = 0;
    
    
    public VisionTrack() {
        requires(Robot.drivetrain);
    }

    protected void execute() {

        double tx = Robot.oi.getxOffset();
        double ty = Robot.oi.getyOffset();

        boolean hasTarget = true;

        double heading_error = -tx;
        
        double distance_error = ty;
        double steering_adjust = 0;


        if(tx > -0.5){
            hasTarget = false;
            steering_adjust = kPAim * heading_error + kDAim*(heading_error - last_heading_error) + min_aim_command;
        } else if (tx < 0.5){
            hasTarget = false;
            steering_adjust = kPAim * heading_error + kDAim*(heading_error - last_heading_error) - min_aim_command;
        } else {
            hasTarget = true;
        }

        last_heading_error = heading_error;

        SmartDashboard.putBoolean("hasTarget", hasTarget);

        double distance_adjust = kPDistance * distance_error;
        DrivetrainSubsystem.drive(Robot.oi.throttleValue() + distance_adjust + steering_adjust, Robot.oi.throttleValue() + distance_adjust - steering_adjust);
    }

    protected boolean isFinished(){
        return false;
    }

    protected void end(){
        DrivetrainSubsystem.drive(0, 0);
    }



}