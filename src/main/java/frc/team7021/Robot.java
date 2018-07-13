package frc.team7021;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.IterativeRobot;

public class Robot extends IterativeRobot {
    TalonSRX motor1 = new TalonSRX(1);
    TalonSRX motor2 = new TalonSRX(4);

    TalonSRX motor3 = new TalonSRX(2);
    TalonSRX motor4 = new TalonSRX(3);

    ITrajectory trajectory;

    public void setSpeed(double left, double right) {
        // Make sure the speeds aren't higher than 1
        left = Math.max(-1, Math.min(left, 1));
        right = Math.max(-1, Math.min(right, 1));

        motor1.set(ControlMode.PercentOutput, -left);
        motor2.set(ControlMode.PercentOutput, -left);

        motor3.set(ControlMode.PercentOutput, right);
        motor4.set(ControlMode.PercentOutput, right);
    }

    public void encConfig() {
        motor2.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        motor4.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    }

    public int getLeftEnc() {
        return motor2.getSelectedSensorPosition(0);
    }
    public int getRightEnc() {
        return motor4.getSelectedSensorPosition(0);
    }

    @Override
    public void autonomousInit() {
        trajectory = new BasicTrajectory();
        encConfig();
    }

    @Override
    public void autonomousPeriodic() {
        trajectory.update(getLeftEnc(), getRightEnc());
        setSpeed(trajectory.getLeftSpeed(), trajectory.getRightSpeed());
    }
}
