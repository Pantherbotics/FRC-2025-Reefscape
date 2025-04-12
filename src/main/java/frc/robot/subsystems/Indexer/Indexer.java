// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  /** Creates a new Passthrough. */
  private final TalonFX m_rollerMotor = new TalonFX(IndexerConstants.kMotorID);

  public Indexer() {
    m_rollerMotor.getConfigurator().apply(IndexerConstants.kMotorConfig);
  }
  public Command setVoltage(double voltage){
    return this.runOnce(()->m_rollerMotor.setVoltage(voltage)).andThen(Commands.idle(this));
  }

  public Current motorCurrent(){
    return m_rollerMotor.getStatorCurrent().getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
