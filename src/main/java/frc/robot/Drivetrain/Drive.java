package frc.robot.Drivetrain;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

//import static frc.robot.Drivetrain.DrivetrainSubsystem.motors;

public class Drive extends Command {
    private static double right, left;
    boolean autoEnabled = false;

    private static final double kJoystickDeadband = 0.03;
    private static final double minVoltMove = 0.8/12.0;

    public Drive(){
        requires(DrivetrainSubsystem.getInstance());
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    protected void execute(){
        
        //Getting the raw joystick values from OI
        double throttle = Robot.oi.throttleValue();
        double turn = Robot.oi.turnValue();

        double quickturnRamp = 0.0;
        
        //Deadbanding the joystick values to avoid moving when there is no input
        throttle = deadbandX(throttle, kJoystickDeadband);
        turn = deadbandX(turn, kJoystickDeadband);
        SmartDashboard.putNumber("stick", DrivetrainSubsystem.rightMotorA.getMotorOutputVoltage());
        SmartDashboard.putNumber("lstick", DrivetrainSubsystem.leftMotorA.getMotorOutputVoltage());

        boolean quickturn = throttle == 0;
        
        if(quickturn){ //Quickturning when no input from throttle stick
            left = 0.5*turn;
            right = -0.5*turn;
        } else { //Binary curvature drive when throttle stick has input, squares outputs to add sensitivity curve
            left = throttle+throttle*turn*0.85;
            right = throttle-throttle*turn*0.85;

            //left = exponentiate(left, 2);
            //right = exponentiate(right, 2);
        }

        SmartDashboard.putNumber("RACurr", DrivetrainSubsystem.rightMotorA.getOutputCurrent());
        SmartDashboard.putNumber("LACurr", DrivetrainSubsystem.leftMotorA.getOutputCurrent());

        left = deadbandY(left, minVoltMove);
        right = deadbandY(right, minVoltMove);

        DrivetrainSubsystem.drive(left, right);
    }

    /**
     * Essentially an implementation of the slope formula, m = (y1-y2)/(x1-x2) = (1 - 0)/(1 - deadband)
     * Uses this slope and multiplies it by the input to deadband a controller from deadband to 1
     * 
     * @param input      an input value to test for deadband
     * @param deadband   deadband value to check the input against
     * @return           returns a double rescaled to fit a deadband
     */
    public static double deadbandX(double input, double deadband){
        if(Math.abs(input) <= deadband){
            return 0;
        } else if(Math.abs(input)==1) {
            return input;
        } else {
            return (1/(1-deadband)*(input+Math.signum(-input)*deadband));
        }
    }

    /**
     * Used to achieve a higher degree of sensitivity when driving
     * Smaller inputs are smaller, larger inputs are larger, without exceeding maximum
     * 
     * @param input   number to put to a power
     * @param power   number to exponentiate input by (not necessarily an integer)
     * @return        returns a double, squared, with the same sign as input
     *     
     */
    public static double exponentiate(double input, double power){
        return Math.copySign(Math.pow(input, power),input);
    }

    /**
     * Essentially deadbandX but in the y direction
     * m = (y1-y2)/(x1-x2) = (1-deadband)(1-0)
     * 
     * @param input      input value to test for deadband 
     * @param deadband   determines values where method returns 0
     * @return           returns a double rescaled according to deadband
     */
    public static double deadbandY(double input, double deadband){
        if(Math.abs(input) == 0.0){
            return 0;
        } else if(Math.abs(input) == 1){
            return input;
        } else {
            return input*(1.0-deadband)+Math.signum(input)*deadband;
        }
    }

}
