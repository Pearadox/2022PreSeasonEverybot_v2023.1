// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static CommandBase ramesete(DriveTrain driveTrain) {
    // Trajectory exampleTrajectory = PathPlanner.loadPath("sphube pickup", new
    // PathConstraints(3, 2),true);
    // List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("sphube
    // pickup", true, new PathConstraints(4, 3));
    // // This is just an example event map. It would be better to have a constant,
    // // global event map
    // // in your code that will be used by all path following commands.
    // HashMap<String, Command> eventMap = new HashMap<>();
    // eventMap.put("IntakeDown", new AutoIntakeDown(_intake));
    // eventMap.put("IntakeIn", new AutoIntakeIn(_intake));
    // eventMap.put("IntakeStop", new IntakeStop(_intake));
    // eventMap.put("IntakeOut", new AutoIntakeOut(_intake));

    // // Create the AutoBuilder. This only needs to be created once when robot code
    // // starts, not every time you want to create an auto command. A good place to
    // // put this is in RobotContainer along with your subsystems.
    // RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
    // _driveTrain::getPose, // Pose2d supplier
    // _driveTrain::resetOdometry, // Pose2d consumer, used to reset odometry at the
    // beginning of auto
    // new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    // DriveTrainConstants.kDriveKinematics, // SwerveDriveKinematics
    // new SimpleMotorFeedforward(
    // DriveTrainConstants.ksVolts,
    // DriveTrainConstants.kvVoltSecondsPerMeter,
    // DriveTrainConstants.kaVoltSecondsSquaredPerMeter),
    // _driveTrain::getWheelSpeeds,
    // new PIDConstants(DriveTrainConstants.kPDriveVel, 0, 0),
    // _driveTrain::tankDriveVolts,
    // eventMap,
    // true, // Should the path be automatically mirrored depending on alliance
    // color.
    // // Optional, defaults to true
    // _driveTrain // The drive subsystem. Used to properly set the requirements of
    // path following
    // // commands

    // );
    // return autoBuilder.fullAuto(pathGroup);
    Trajectory exampleTrajectory = PathPlanner.loadPath("sphube pickup", new PathConstraints(2, 1), true);
    // var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
    //     new SimpleMotorFeedforward(
    //         DriveTrainConstants.ksVolts,
    //         DriveTrainConstants.kvVoltSecondsPerMeter,
    //         DriveTrainConstants.kaVoltSecondsSquaredPerMeter),
    //     DriveTrainConstants.kDriveKinematics,
    //     10);

    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     2.0,
    //     1.5)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveTrainConstants.kDriveKinematics)
    //     // Apply the voltage constraint
    //     .addConstraint(autoVoltageConstraint);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     // Pass config
    //     config);

    driveTrain.resetEncoders();
    driveTrain.zeroHeading();
    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        driveTrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            DriveTrainConstants.ksVolts,
            DriveTrainConstants.kvVoltSecondsPerMeter,
            DriveTrainConstants.kaVoltSecondsSquaredPerMeter),
        DriveTrainConstants.kDriveKinematics,
        driveTrain::getWheelSpeeds,
        new PIDController(DriveTrainConstants.kPDriveVel, 0, 0),
        new PIDController(DriveTrainConstants.kPDriveVel, 0, 0),
        driveTrain::tankDriveVolts,
        driveTrain);
    driveTrain.resetOdometry(exampleTrajectory.getInitialPose());
    return ramseteCommand;
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
