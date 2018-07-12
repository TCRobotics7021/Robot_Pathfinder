package frc.team7021;

public interface ITrajectory {
    void update(int leftEnc, int rightEnc);

    double getLeftSpeed();

    double getRightSpeed();
}
