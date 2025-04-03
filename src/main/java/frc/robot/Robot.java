// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.robotState.ToggleAllianceColorCommand;
import frc.robot.constants.BuildConstants;
import frc.robot.subsystems.robotState.RobotStateSubsystem.ScoreSide;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    if (isReal()) {
      Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
      Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
      Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
      Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
      Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
      switch (BuildConstants.DIRTY) {
        case 0:
          Logger.recordMetadata("GitDirty", "All Changes Committed");
          break;
        case 1:
          Logger.recordMetadata("GitDirty", "Uncommitted changes");
          break;
        default:
          Logger.recordMetadata("GitDirty", "Unknown");
          break;
      }
      Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
      Logger.addDataReceiver(new NT4Publisher());
    }
    Logger.start();

    Shuffleboard.getTab("Match")
        .add(new ToggleAllianceColorCommand(m_robotContainer))
        .withSize(1, 1)
        .withPosition(2, 0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.stopTagAlign();
  }

  @Override
  public void disabledPeriodic() {
    // if (!m_robotContainer.hasBiscuitZeroed()) m_robotContainer.zeroBiscuit();

    // if (!m_robotContainer.hasSwerveZeroed()) m_robotContainer.zeroSwerve();

    m_robotContainer.getAutoSwitch().checkSwitch();
    m_robotContainer.updateCanivoreStatus();
    m_robotContainer.updateCANErrorCount();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.setIsAuto(true);
    if (m_robotContainer.getAutoSwitch().getAutoCommand() != null) {
      m_robotContainer.getAutoSwitch().getAutoCommand().schedule();
    }
    m_robotContainer.setHeadlights(true);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if (!m_robotContainer.hasElevatorZeroed()) m_robotContainer.zeroElevator();
    else if (m_robotContainer.wasScoringCoral()) {
      m_robotContainer.finishAuto();
    } else {
      m_robotContainer.stow();
    }

    m_robotContainer.setIsAuto(false);
    m_robotContainer.setIsAutoPlacing(true);
    m_robotContainer.setScoringSide(ScoreSide.LEFT);
    m_robotContainer.disableNoMotionCal();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
