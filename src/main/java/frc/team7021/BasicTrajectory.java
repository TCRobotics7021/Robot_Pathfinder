package frc.team7021;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class BasicTrajectory implements ITrajectory {

    double P = 0;
    double I = 0;
    double D = 0;

    // Robot Properties
    int encoderPPRev = 600;
    double wheelDiameter = 0.174625;
    double width = 0.50165;

    // Used to scale motor speed in encoder ticks per second
    double maxVelocityLeftEnc = 3245.9101532061695;
    double maxVelcoityRightEnc = 3310.8283562702927;

    double maxVelocityLeft = 0.5; // maxVelocityLeftEnc * wheelDiameter * Math.PI / encoderPPRev;
    double maxVelocityRight = 0.5; // maxVelcoityRightEnc * wheelDiameter * Math.PI / encoderPPRev;

    // These values are used to generate the trajectory
    double dt = 0.2;
    double maxTravelVelocity = 0.1;
    double maxAccel = 10;
    double maxJerk = 60;

    EncoderFollower left;
    EncoderFollower right;

    double leftSpeed = 0;
    double rightSpeed = 0;

    // Called just before this Command runs the first time
    public BasicTrajectory() {
        // Specify path
        Waypoint[] points = new Waypoint[]{
                new Waypoint(0, 0, 0),
                new Waypoint(1, 0, Pathfinder.d2r(45)),
                new Waypoint(1, 1, Pathfinder.d2r(90))
        };


        // Configure trajectory
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
                Trajectory.Config.SAMPLES_HIGH, dt, maxTravelVelocity, maxAccel, maxJerk);
        Trajectory trajectory = Pathfinder.generate(points, config);

        TankModifier modifier = new TankModifier(trajectory).modify(width);

        left = new EncoderFollower(modifier.getLeftTrajectory());
        right = new EncoderFollower(modifier.getRightTrajectory());

        int encoder_position_left = 0;
        int encoder_position_right = 0;

        // Configure encoder followers
        left.configureEncoder(encoder_position_left, encoderPPRev, wheelDiameter);
        left.configurePIDVA(P, I, D, 1 / maxVelocityLeft, maxAccel);

        right.configureEncoder(encoder_position_right, encoderPPRev, wheelDiameter);
        right.configurePIDVA(P, I, D, 1 / maxVelocityRight, maxAccel);
    }

    public void update(int leftEnc, int rightEnc) {
        leftSpeed = left.calculate(leftEnc);
        rightSpeed = right.calculate(rightEnc);
    }

    public double getLeftSpeed() {
        return leftSpeed;
    }

    public double getRightSpeed() {
        return rightSpeed;
    }
}
