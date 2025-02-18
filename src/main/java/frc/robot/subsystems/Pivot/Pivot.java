// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Pivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  private final TalonFX m_pivotMotor = new TalonFX(PivotConstants.kPivotMotorID);
  private final CANcoder m_pivotEncoder = new CANcoder(PivotConstants.kEncoderID);

  private Angle goalAngle = Degrees.of(0);
  private final MotionMagicExpoVoltage m_motionMagicReq = new MotionMagicExpoVoltage(0).withEnableFOC(true);

  private final VoltageOut m_voltReq = new VoltageOut(0);
  private final SysIdRoutine routine = new SysIdRoutine(
    new Config( Volts.of(0.75).per(Second), Volts.of(2), null, (state)->SignalLogger.writeString("State", state.toString())),
    new Mechanism(this::setVolts, null, this));

  public Pivot() {
    m_pivotMotor.getConfigurator().apply(PivotConstants.kPivotMotorConfigs);
    m_pivotEncoder.getConfigurator().apply(PivotConstants.kEncoderConfigs);

    BaseStatusSignal.setUpdateFrequencyForAll(250,
      m_pivotMotor.getPosition(),
      m_pivotMotor.getVelocity(),
      m_pivotMotor.getMotorVoltage(),
      m_pivotMotor.getTorqueCurrent(),
      m_pivotEncoder.getPosition(),
      m_pivotEncoder.getVelocity());

  }

  public Command setAngleCommand(Angle angle){
    return this.runOnce(()->{
      goalAngle = Degrees.of(MathUtil.clamp(angle.in(Degrees), PivotConstants.kMinAngle.in(Degrees), PivotConstants.kMaxAngle.in(Degrees))).minus(Degrees.of(90));
      SmartDashboard.putNumber("Pivot commanded angle", goalAngle.in(Degrees));
      m_pivotMotor.setControl(m_motionMagicReq.withPosition(goalAngle));
    }).andThen(Commands.idle())
    .until(this::isAtGoal);
  }

  public Command sysIdDynamicCommand(Direction direction){
    return routine.dynamic(direction);
  }
  public Command sysIdQuasistaticCommand(Direction direction){
    return routine.quasistatic(direction);
  }

  public boolean isAtGoal(){
    return pivotAngle().isNear(goalAngle, PivotConstants.kGoalTolerance);
  }

  public Angle pivotAngle(){
    return m_pivotMotor.getPosition().getValue();
  }

  private void setVolts(Voltage volts){
    m_pivotMotor.setControl(m_voltReq.withOutput(volts));
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("pivotAtGoal", isAtGoal());
    SmartDashboard.putNumber("pivot angle", pivotAngle().in(Degrees));
    SmartDashboard.putNumber("pivot goal", goalAngle.in(Degrees));
  }
}
