// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import au.grapplerobotics.CanBridge;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.RobotStates;
import frc.robot.subsystems.Vision.Vision;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    CanBridge.runTCP();
    Vision.setup();
    RobotStates.setupPositionTable();
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    Vision.updateCamera();
    if (Vision.leftEstimatedPose.isPresent()){
      var leftPose = Vision.leftEstimatedPose.get();
      m_robotContainer.addVisionMeasurement(leftPose.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(leftPose.timestampSeconds));
    }
    if (Vision.rightEstimatedPose.isPresent()){
      var rightPose = Vision.leftEstimatedPose.get();
      m_robotContainer.addVisionMeasurement(rightPose.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(rightPose.timestampSeconds));
    }

    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
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
