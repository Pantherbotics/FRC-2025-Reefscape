// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeRollerConstants;

public class AlgaeRoller extends SubsystemBase {
  private final TalonFXS m_rollerMotor = new TalonFXS(AlgaeRollerConstants.kMotorID);
  private final VoltageOut m_voltReq = new VoltageOut(Volts.of(0));
  /** Creates a new AlgaeRoller. */
  public AlgaeRoller() {
    m_rollerMotor.getConfigurator().apply(AlgaeRollerConstants.kRollerMotorConfigs);
    SmartDashboard.putData(this);
  }

  public Command setVoltage(Voltage voltage){
    return this.runOnce(()->m_rollerMotor.setControl(m_voltReq.withOutput(voltage)))
    .andThen(Commands.idle(this));
  }

  public Command runUntilCurrent(Voltage voltage, Current current){
    return setVoltage(voltage).until(()->m_rollerMotor.getStatorCurrent().getValue().gt(current));
  }

  @Override
  public void periodic() {
  }
}
