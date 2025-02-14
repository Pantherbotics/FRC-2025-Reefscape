// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Pivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
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
  private final MotionMagicExpoTorqueCurrentFOC m_motionMagicReq = new MotionMagicExpoTorqueCurrentFOC(Degrees.zero());

  private final TorqueCurrentFOC m_torqueReq = new TorqueCurrentFOC(Amps.zero());
  private final SysIdRoutine routine = new SysIdRoutine(
    new Config(null, Volts.of(4), null, (state)->SignalLogger.writeString("State", state.toString())),
    new Mechanism((output)->setCurrent(Amps.of(output.in(Volts))), null, this));

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
      goalAngle = Rotations.of(MathUtil.clamp(angle.in(Degrees), PivotConstants.kMinAngle.in(Degrees), PivotConstants.kMaxAngle.in(Degrees)));
      m_pivotMotor.setControl(m_motionMagicReq.withPosition(angle));
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
    return m_pivotMotor.getPosition().getValue().isNear(goalAngle, PivotConstants.kGoalTolerance);
  }

  private void setCurrent(Current current){
    m_pivotMotor.setControl(m_torqueReq.withOutput(current));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
