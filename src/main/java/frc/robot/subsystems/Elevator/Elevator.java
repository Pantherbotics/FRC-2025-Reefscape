package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private final TalonFX m_leaderMotor = new TalonFX(ElevatorConstants.kLeaderMotorID);
  private final TalonFX m_followerMotor = new TalonFX(ElevatorConstants.kFollowerMotorID);
  private Distance goalHeight = Inches.of(0);
  private final MotionMagicExpoTorqueCurrentFOC m_motionMagicReq = new MotionMagicExpoTorqueCurrentFOC(0);
  
  private final VoltageOut m_voltReq = new VoltageOut(0);
  private final SysIdRoutine routine = new SysIdRoutine(
    new Config(null, Volts.of(3), null, state -> SignalLogger.writeString("state", state.toString())), 
    new Mechanism(this::setVolt, null, this)
  );

  public Elevator () {
    m_leaderMotor.getConfigurator().apply(ElevatorConstants.kElevatorConfigs);
    m_followerMotor.getConfigurator().apply(ElevatorConstants.kFolllowerConfigs);
    m_followerMotor.setControl(new Follower(ElevatorConstants.kLeaderMotorID, true));

    BaseStatusSignal.setUpdateFrequencyForAll(250,
    m_leaderMotor.getPosition(),
    m_leaderMotor.getVelocity(),
    m_leaderMotor.getMotorVoltage(),
    m_leaderMotor.getTorqueCurrent());

    SmartDashboard.putData(this);
  }

  public Command zeroEncoder(){
    return this.startEnd(
      ()->m_leaderMotor.setControl(new VoltageOut(ElevatorConstants.kZeroVoltage)),
      ()->m_leaderMotor.setControl(new NeutralOut())
    ).finallyDo(
      ()->m_leaderMotor.setPosition(Degrees.of(0))//(interrupt)->{ if (!interrupt){ m_leaderMotor.setPosition(Degrees.of(0)); } }
    ).until(
      ()->m_followerMotor.getStatorCurrent().getValue().gt(Amps.of(7))
    );
  }


  public Command setHeightCommand(Distance height){

    return this.runOnce(()->{
      SmartDashboard.putNumber("commanded height", height.in(Inches));
      goalHeight = Inches.of(MathUtil.clamp(height.in(Inches), 0, ElevatorConstants.kElevatorMaxHeight.in(Inches)));
      Angle goalAngle = goalHeight.timesConversionFactor(ElevatorConstants.kConversion.reciprocal());
      m_leaderMotor.setControl(m_motionMagicReq.withPosition(goalAngle));
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
    //return MathUtil.isNear(goalHeight.in(Inches), m_leaderMotor.getPosition().getValueAsDouble() * ElevatorConstants.kElevatorRatio * ElevatorConstants.kSprocketPitchCircumference.in(Inches),0.1);
    return m_leaderMotor.getPosition().getValue().timesConversionFactor(ElevatorConstants.kConversion).isNear(goalHeight, ElevatorConstants.kGoalTolerance);
  }

  private void setVolt(Voltage volts){
    m_leaderMotor.setControl(m_voltReq.withOutput(volts));
  }

  // unused periodic method
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("isatgoal", isAtGoal());
    SmartDashboard.putNumber("goalHeight", goalHeight.in(Inches));
    SmartDashboard.putNumber("Position", m_leaderMotor.getPosition().getValue().timesConversionFactor(ElevatorConstants.kConversion).in(Inches));
  }
}

