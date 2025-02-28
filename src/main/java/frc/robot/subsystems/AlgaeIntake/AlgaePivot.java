// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.AlgaePivotConstants;

public class AlgaePivot extends SubsystemBase {
  private final TalonFX m_pivotMotor = new TalonFX(AlgaePivotConstants.kMotorID);
  private final MotionMagicExpoVoltage m_MotionMagicReq = new MotionMagicExpoVoltage(0).withEnableFOC(true);
  private Angle m_goalAngle = Degrees.of(90);

  private final VoltageOut m_voltReq = new VoltageOut(0);

  private final SysIdRoutine routine = new SysIdRoutine(
    new Config( null, Volts.of(6), null, (state)->SignalLogger.writeString("State", state.toString())),
    new Mechanism(this::setVolts, null, this));

  public AlgaePivot() {
    m_pivotMotor.getConfigurator().apply(AlgaePivotConstants.kPivotConstants);
    m_pivotMotor.setPosition(Degrees.of(90));
    SmartDashboard.putData(this);

    BaseStatusSignal.setUpdateFrequencyForAll(100, 
      m_pivotMotor.getVelocity(),
      m_pivotMotor.getPosition(),
      m_pivotMotor.getMotorVoltage()
    );

  }

  public Angle getAngle(){
    return m_pivotMotor.getPosition().getValue();
  }

  public Command setAngleCommand(Angle angle){
    return this.runOnce(()->{
      m_goalAngle = Degrees.of(MathUtil.clamp(angle.in(Degrees), AlgaePivotConstants.kMinAngle.in(Degrees), AlgaePivotConstants.kMaxAngle.in(Degrees)));
      m_pivotMotor.setControl(m_MotionMagicReq.withPosition(angle));
    }).andThen(Commands.idle(this)).until(this::isAtGoal);
  }
  private void setVolts(Voltage volts){
    m_pivotMotor.setControl(m_voltReq.withOutput(volts));
  }

  public Command sysIdDynamicCommand(Direction direction){
    return routine.dynamic(direction);
  }
  
  public Command sysIdQuasistaticCommand(Direction direction){
    return routine.quasistatic(direction);
  }

  public boolean isAtGoal(){
    return m_pivotMotor.getPosition().getValue().isNear(m_goalAngle, AlgaePivotConstants.kPositionTolerance);
  }
  public void meaningOfLife(){}

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("AlgaeAtGoal", isAtGoal());
    SmartDashboard.putNumber("AlgaePivot", m_pivotMotor.getPosition().getValue().in(Degrees));
  }
}
