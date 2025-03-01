// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.lang.model.util.ElementScanner6;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.RobotStates;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  // private Vision vision;

  public Robot() {
    CanBridge.runTCP();

    // vision = new Vision();
    // Vision.setup();
    RobotStates.setupPositionTable();
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void robotPeriodic() {
    m_robotContainer.updateVision();

        // Correct pose estimate with vision measurements
        
    // Vision.updateCamera();
    // if (Vision.leftEstimatedPose.isPresent()){
    //   var leftPose = Vision.leftEstimatedPose.get();
    //   m_robotContainer.addVisionMeasurement(leftPose.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(leftPose.timestampSeconds), Vision.getCurrentStdDev());
    // }
    // if (Vision.rightEstimatedPose.isPresent()){
    //   var rightPose = Vision.rightEstimatedPose.get();
    //   m_robotContainer.addVisionMeasurement(rightPose.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(rightPose.timestampSeconds));
    // } 

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
    } else {
      m_robotContainer.moveAuto().schedule();;
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
  public void teleopPeriodic() {

  }

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
   
  @Override
  public void simulationPeriodic(){
    m_robotContainer.updateSimVision();
  }

}