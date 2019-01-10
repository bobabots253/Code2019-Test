package frc.robot.Autonomous;

import java.io.File;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Drivetrain.DrivetrainSubsystem;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class PathFollower extends Command{

    //Constant values for PIDVA correction
    private double kP = 0, kI = 0, kD = 0, kV = 1/2.872716583788768, kA = 0;
    private double maxSpeed = 2, maxAccel = 1, maxJerk = 15; //These only apply to the Waypoint[] constructor

    Trajectory trajecLeft, trajecRight;
    EncoderFollower followerLeft, followerRight;

    //Robot measurements (in meters)
    private static double kWheelDiameter = 0.1524; //6 in.
    private static double kWheelbase = 0.59817;

    //Constructor for command that takes a String path name
    public PathFollower(String pathName){

        DrivetrainSubsystem.resetEncoders();

        requires(Robot.drivetrain);

        trajecLeft = Pathfinder.readFromCSV(new File("/home/lvuser/profiles/" + pathName + "_left.csv"));
        trajecRight = Pathfinder.readFromCSV(new File("/home/lvuser/profiles/" + pathName + "_right.csv"));
    }

    //Constructor for command that takes a Waypoint array object
    public PathFollower(Waypoint[] points){

        requires(Robot.drivetrain);

        Trajectory.Config pointsConfig = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_LOW, 0.02, maxSpeed, maxAccel, maxJerk);
        Trajectory pointsTrajec = Pathfinder.generate(points, pointsConfig);

        TankModifier modifier = new TankModifier(pointsTrajec);
        modifier.modify(kWheelbase);

        trajecLeft = modifier.getLeftTrajectory();
        trajecRight = modifier.getRightTrajectory();
    }

    //This method runs only once when the Command is initialized
    protected void initialize(){
        //Resets current heading and encoder values to avoid errors
        DrivetrainSubsystem.resetGyro();
        DrivetrainSubsystem.resetEncoders();
        DrivetrainSubsystem.setBrakeMode();

        //Creating EncoderFollower objects from Trajectory objects in constructors
        followerLeft = new EncoderFollower(trajecLeft);
        followerRight = new EncoderFollower(trajecRight);

        //Sets encoders for error calculation
        followerLeft.configureEncoder(DrivetrainSubsystem.leftMotorA.getSelectedSensorPosition(0),
                4517, kWheelDiameter);
        followerRight.configureEncoder(DrivetrainSubsystem.rightMotorA.getSelectedSensorPosition(0),
                4517, kWheelDiameter);

        //Configures PIDVA values 
        followerLeft.configurePIDVA(kP, kI, kD, kV, kA);
        followerRight.configurePIDVA(kP, kI, kD, kV, kA);

    }

    //execute() is called every 20 ms (RoboRIO default loop rate) 
    protected void execute() {

        //Calculates left and right motor outputs based on a given encoder value 
        double left = followerLeft.calculate(DrivetrainSubsystem.leftMotorA.getSelectedSensorPosition(0));
        double right = followerRight.calculate(DrivetrainSubsystem.rightMotorA.getSelectedSensorPosition(0));

        //left = processIntercept(left, 0.968);
        //right = processIntercept(right, 1.058);

        //Gyro proportional correction
        double gyroHeading = -DrivetrainSubsystem.gyro.getAngle(); //Inverts gyro to make it left hand positive like Pathfinder
        SmartDashboard.putNumber("Path Gyro Heading", gyroHeading);
        double desiredHeading = Pathfinder.r2d(followerRight.getHeading());
        double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);
        double turn = .8 * (-1.0/80.0) * angleDifference;

        double leftspeed = left+turn + 0.968/12;
        double rightspeed = right-turn + 1.058/12;

        
        //Checks if the follower is finished before calling .getSegment() to avoid runtime errors
        if(!followerLeft.isFinished()){
            SmartDashboard.putNumber("Path left enc error", toTicks(followerLeft.getSegment().position)-DrivetrainSubsystem.leftMotorA.getSelectedSensorPosition(0));
            SmartDashboard.putNumber("Path right enc error", toTicks(followerRight.getSegment().position)-DrivetrainSubsystem.rightMotorA.getSelectedSensorPosition(0));
            //SmartDashboard.putNumber("Left path encoder", DrivetrainSubsystem.leftMotorA.getSelectedSensorPosition(0));
            //SmartDashboard.putNumber("Right path encoder", DrivetrainSubsystem.rightMotorA.getSelectedSensorPosition(0)); 
            SmartDashboard.putNumber("Path Position Right", followerRight.getSegment().position);
            SmartDashboard.putNumber("Path Position Left", followerLeft.getSegment().position);
            SmartDashboard.putNumber("Path Velocity Left", followerLeft.getSegment().velocity);
            SmartDashboard.putNumber("Path Velocity Right", followerRight.getSegment().velocity);
        }

        SmartDashboard.putNumber("Path commanded left speed", leftspeed);
        SmartDashboard.putNumber("Path commanded right speed", rightspeed);

        SmartDashboard.putNumber("Robot Position Right", toMeters(DrivetrainSubsystem.rightMotorA.getSelectedSensorPosition(0)));
        SmartDashboard.putNumber("Robot Position Left", toMeters(DrivetrainSubsystem.leftMotorA.getSelectedSensorPosition(0)));

        SmartDashboard.putNumber("Robot Velocity Right", toMeters(DrivetrainSubsystem.rightMotorA.getSelectedSensorVelocity(0)));
        SmartDashboard.putNumber("Robot Velocity Left", toMeters(DrivetrainSubsystem.leftMotorA.getSelectedSensorVelocity(0)));

        
        DrivetrainSubsystem.drive(leftspeed, rightspeed); //Drives at calculated speeds
    }

    /* Uses dimensional analysis to convert meters to encoder ticks
     *
     *          39.3701 in     1 revolution       4517 encoder ticks
     * meter * ------------ * --------------- * ----------------------
     *           1 meter        6pi inches           1 revolution
     */
    public static double toTicks(double meters){
        return ((meters*39.3701)/(6*Math.PI))*4517;
    }

    public static double toMeters(double ticks){
        return ((ticks/4517.0)*6*Math.PI)*254;
    }

    public static double processIntercept(double commandedVoltage, double interceptVoltage){
        return commandedVoltage + interceptVoltage;
    }

    @Override
    protected boolean isFinished() { return followerLeft.isFinished() && followerRight.isFinished(); } //Command is finished when both followers are finished

}