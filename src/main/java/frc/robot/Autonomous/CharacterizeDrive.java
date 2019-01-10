package frc.robot.Autonomous;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.apache.commons.math3.stat.regression.SimpleRegression;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Drivetrain.DrivetrainSubsystem;

public class CharacterizeDrive extends Command {
    

    static class dtSide {

        public double percentOut = 0;
        public double vIntercept = 0;
        public SimpleRegression regression = new SimpleRegression();

        public String side;
        public double speed;
        public ArrayList<Triple> dataPoints = new ArrayList<Triple>();

        public TalonSRX speedController;

        public dtSide(String side, TalonSRX talon){
            this.side = side;
            this.speedController = talon;

            updateEncoder();
        }

        public void updateEncoder(){
            this.speed = speedController.getSelectedSensorVelocity(0);
        }

        public void addPoint(){
            updateEncoder();
            dataPoints.add(new Triple(percentOut, speed, Timer.getFPGATimestamp()));
        }

        public void setPercentOut(double percentOut){
            this.percentOut = percentOut;
        }

    }

    static final class Triple{
        public final double x;
        public final double y;
        public final double z;

        public Triple(double x, double y, double z){
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public boolean equals(Triple arg){
            if(arg.x == this.x && arg.y == this.y && arg.z == this.z){
                return true;
            } else {
                return false;
            }
        }
    }

    private final Timer timer = new Timer();
    private static final dtSide leftSide = new dtSide("Left", DrivetrainSubsystem.leftMotorA);
    private static final dtSide rightSide = new dtSide("Right", DrivetrainSubsystem.rightMotorA);
    private static final dtSide[] sides = {leftSide, rightSide};

    public CharacterizeDrive(){
        requires(Robot.drivetrain);
        setInterruptible(false);
    }

    protected void initialize(){
        applySpeeds();
        for(dtSide side : sides){
            if(side.speed >= 0.1){
                side.addPoint();
            }
        }
        timer.start();
    }

    private static void applySpeeds(){
        DrivetrainSubsystem.drive(leftSide.percentOut, rightSide.percentOut);
    }

    protected void execute(){
        if(timer.hasPeriodPassed(1.0)){
            for(dtSide side : sides){
                side.setPercentOut(side.percentOut += 0.25/12);
                applySpeeds();
            }
            timer.reset();
        }
    }

    protected boolean isFinished(){
        return(leftSide.percentOut >= 0.3 && rightSide.percentOut >= 0.3);
    }

    protected void end(){
        DrivetrainSubsystem.drive(0, 0);

        for(dtSide side : sides){
            for(Triple dataPoints : side.dataPoints){
                side.regression.addData(dataPoints.x, dataPoints.y);
            }
            System.out.println(side.side + " side: Slope: " + side.regression.getSlope() + ", Intercept: " + side.regression.getIntercept() + ", Linearity: " + side.regression.getRSquare());
        }
    }

}
