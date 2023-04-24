// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1515.botmitzvah;

import org.team1515.botmitzvah.Utils.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.team364.swervelib.util.SwerveConstants;
import com.team364.swervelib.util.SwerveConstants.AutoConstants;
import com.team364.swervelib.util.SwerveConstants.Swerve;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.team1515.botmitzvah.Commands.*;
import org.team1515.botmitzvah.Subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.XboxController;

public class RobotContainer {
  public static XboxController mainController;
  public static XboxController secondController;

  public static Drivetrain drivetrain;
  public static Gyroscope gyro;
  public static SwerveAutoBuilder swerveAutoBuilder;

  public RobotContainer() {
    mainController = new XboxController(0);

    gyro = new Gyroscope();
    drivetrain = new Drivetrain(new Pose2d());
    Map<String, Command> eventMap = new HashMap<>();

    swerveAutoBuilder = new SwerveAutoBuilder(
        drivetrain::getRobotPose, // Pose2d supplier
        drivetrain::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        SwerveConstants.Swerve.swerveKinematics, // SwerveDriveKinematics
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        drivetrain::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
    );

    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        new DefaultDriveCommand(drivetrain,
            () -> -modifyAxis(-mainController.getLeftY() * getRobotSpeed()),
            () -> -modifyAxis(-mainController.getLeftX() * getRobotSpeed()),
            () -> modifyAxis(mainController.getRightX() * getRobotSpeed()),
            () -> Controls.DRIVE_ROBOT_ORIENTED.getAsBoolean()));

    Controls.RESET_GYRO.onTrue(new InstantCommand(() -> drivetrain.zeroGyro()));
  }

  public Command getAutonomousCommand() {
    
    PathPlannerTrajectory traj1 = PathPlanner.generatePath(
      new PathConstraints(1, 1), // max speed and accel in m/s and m/s^2 
      new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0)),
      new PathPoint(new Translation2d(3.0, 0.0), Rotation2d.fromDegrees(45))
    );

    Command swerveAutoCommand = swerveAutoBuilder.fullAuto(traj1);

    return swerveAutoCommand.andThen(() -> drivetrain.drive(new Translation2d(0, 0), 0, false, true));
  }

  public static double getRobotSpeed() {
    return Controls.getLeftTrigger() ? 0.45 : 0.7;
  }

  private static double modifyAxis(double value) {
    value = Utilities.deadband(value, 0.08);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}