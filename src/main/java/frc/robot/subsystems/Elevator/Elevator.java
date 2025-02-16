package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
  private final MotionMagicExpoVoltage m_motionMagicReq = new MotionMagicExpoVoltage(0).withEnableFOC(true);
  
  private final VoltageOut m_voltReq = new VoltageOut(0).withEnableFOC(true);
  private final SysIdRoutine routine = new SysIdRoutine(
    new Config(null, Volts.of(3), null, state -> SignalLogger.writeString("state", state.toString())), 
    new Mechanism(this::setVolts, null, this)
  );
  private boolean hasZeroedSinceBoot = false;
  private final Alert moveUnzeroed = new Alert("Elevator not zeroed!", AlertType.kWarning);

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
      ()->{m_leaderMotor.setPosition(Degrees.of(0)); hasZeroedSinceBoot = true; moveUnzeroed.set(false);}//(interrupt)->{ if (!interrupt){ m_leaderMotor.setPosition(Degrees.of(0)); } }
    ).until(
      ()->m_followerMotor.getStatorCurrent().getValue().gt(Amps.of(7))
    );
  }


  public Command setHeightCommand(Distance height){
    if (hasZeroedSinceBoot){
    return this.runOnce(()->{
      SmartDashboard.putNumber("Elevator commanded height", height.in(Inches));
      goalHeight = Inches.of(MathUtil.clamp(height.in(Inches), 0, ElevatorConstants.kElevatorMaxHeight.in(Inches)));
      Angle goalAngle = goalHeight.timesConversionFactor(ElevatorConstants.kConversion.reciprocal());
      
      m_leaderMotor.setControl(m_motionMagicReq.withPosition(goalAngle));
    }).andThen(Commands.idle())
      .until(this::isAtGoal);
    } else {
      return this.runOnce(()->moveUnzeroed.set(true));
    }
  }

  public Command sysIdDynamicCommand(Direction direction){
    return routine.dynamic(direction);
  }
  public Command sysIdQuasistaticCommand(Direction direction){
    return routine.quasistatic(direction);
  }
  
  public boolean isAtGoal(){
    return elevatorPosition().isNear(goalHeight, ElevatorConstants.kGoalTolerance);
  }

  public Distance elevatorPosition(){
    return m_leaderMotor.getPosition().getValue().timesConversionFactor(ElevatorConstants.kConversion);
  }

  private void setVolts(Voltage volts){
    m_leaderMotor.setControl(m_voltReq.withOutput(volts));
  }

  // unused periodic method
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("ElevatorAtGoal", isAtGoal());
    SmartDashboard.putNumber("ElevatorPosition", elevatorPosition().in(Inches));
  }
}

