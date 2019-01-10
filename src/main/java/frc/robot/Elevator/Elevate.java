package frc.robot.Elevator;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Drivetrain.Drive;
import frc.robot.OI.OI;

public class Elevate extends Command {

    public static double holdVoltage = 0.15;

    public Elevate(){
        requires(Robot.elevator);
    }

    protected void execute(){
        double elevateAxis = OI.elevateValue(); //sets elevateAxis to joystick input
        elevateAxis = Drive.deadbandX(elevateAxis, 0.05);

        SmartDashboard.putNumber("Elevator Joystick Input", elevateAxis);
        SmartDashboard.putNumber("Elevator encoder", ElevatorSubsystem.getEncoder());
        SmartDashboard.putNumber("Elevator output bottom", ElevatorSubsystem.elevAVoltage());
      //  SmartDashboard.putNumber("Elevator output top", ElevatorSubsystem.elevBVoltage());

        if(elevateAxis != 0){
            ElevatorSubsystem.elevate(elevateAxis);
        } else {
            ElevatorSubsystem.elevate(holdVoltage);
        }
    }


    @Override
    protected boolean isFinished() {
        return false;
    }

    
}
