// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.RobotStates;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Timer loopTimer;

  private final RobotContainer m_robotContainer;
  // private Vision vision;

  public Robot() {
    loopTimer = new Timer();
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
    loopTimer.reset();
    loopTimer.start();
    m_robotContainer.updateVision();
    CommandScheduler.getInstance().run();
    m_robotContainer.visualizer.update();

      loopTimer.stop();
      double loopTime = loopTimer.get();
    SmartDashboard.putNumber("Loop time", loopTime);
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
      if (m_autonomousCommand.getRequirements().size() == 0){
        m_robotContainer.moveAuto().schedule();
      } else {
        System.out.println(m_autonomousCommand.getRequirements().size());
        m_autonomousCommand.schedule();
      }
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
    CommandScheduler.getInstance().cancelAll(); 
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