package frc.robot.OI;

import static frc.robot.Drivetrain.DrivetrainSubsystem.leftMotorB;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;


public class ElevatorEncoderSource implements PIDSource {
    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {

    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return null;
    }

    @Override
    public double pidGet() {
        return leftMotorB.getSelectedSensorPosition(0);
    }
}
