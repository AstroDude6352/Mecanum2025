package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;

public class MoveToTarget extends Command {
    private final LimelightSubsystem limelightSubsystem;
    private final MecanumDrive robotDrive;
    private final PIDController aimPIDController;
    private final PIDController rangePIDController;

    private static final double MAX_SPEED = 0.3; // Maximum speed
    private static final double MAX_ROTATION = 0.3; // Maximum rotation speed
    private static final double DEADBAND = 0.05; // Deadband for small errors

    public MoveToTarget(LimelightSubsystem subsystem, MecanumDrive robotDrive) {
        this.limelightSubsystem = subsystem;
        this.robotDrive = robotDrive;
        addRequirements(subsystem);

        // Initialize PID controllers with adjusted gains
        aimPIDController = new PIDController(0.03, 0.01, 0.02);
        rangePIDController = new PIDController(0.1, 0.01, 0);

        // Set tolerances if needed
        aimPIDController.setTolerance(1.0);
        rangePIDController.setTolerance(1.0);
    }

    @Override
    public void execute() {
        if (limelightSubsystem.hasTarget()) {
            double rot = aimPIDController.calculate(limelightSubsystem.getX(), 0);
            double forward = rangePIDController.calculate(limelightSubsystem.getA(), LimelightSubsystem.targetArea);

            // Apply deadband
            if (Math.abs(rot) < DEADBAND) {
                rot = 0;
            }
            if (Math.abs(forward) < DEADBAND) {
                forward = 0;
            }

            // Limit the maximum speed and rotation
            rot = Math.max(Math.min(rot, MAX_ROTATION), -MAX_ROTATION);
            forward = Math.max(Math.min(forward, MAX_SPEED), -MAX_SPEED);

            // Adjust strafing based on rotation direction
            double strafe = forward * (rot > 0 ? -1.0 : 1.0);

            robotDrive.driveCartesian(forward * -0.15, strafe * 0.15, rot * 0.3);
        } else {
            robotDrive.driveCartesian(0, 0, 0); // Stop the robot if no target is found
        }
    }

    @Override
    public void end(boolean interrupted) {
        robotDrive.driveCartesian(0, 0, 0); // Stop the robot
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}