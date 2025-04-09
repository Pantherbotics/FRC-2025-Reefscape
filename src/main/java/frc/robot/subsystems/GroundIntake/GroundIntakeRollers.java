// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GroundIntake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntakeRollerConstants;
import frc.robot.Constants.IndexerConstants;

public class GroundIntakeRollers extends SubsystemBase {
  /** Creates a new GroundIntake. */
  private final TalonFX m_rollerMotor = new TalonFX(GroundIntakeRollerConstants.kMotorID);
  public GroundIntakeRollers() {
    m_rollerMotor.getConfigurator().apply(GroundIntakeRollerConstants.kMotorConfig);
  }

  public Command setVoltage(double voltage){
    return this.runOnce(()->m_rollerMotor.setVoltage(voltage)).andThen(Commands.idle(this));
  }

  @Override
  public void periodic() {}
}
