// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.PIDControllers;

import java.util.Optional;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;

/** Makes it super easy to tune, allows to change K values from smart dashboard*/
/** Makes it super easy to tune controllers, allows programmers to change constants from smart dashboard */

public class SmartPIDControllerTalonFX implements SmartPIDBase {

  public String name;
  public boolean smart;
  public TalonFX motor;
  public Optional<Double> lastKvValue0 = Optional.empty();
  public Optional<Double> lastKaValue0 = Optional.empty();
  public Optional<Double> lastKsValue0 = Optional.empty();
  public Optional<Double> lastKvValue1 = Optional.empty();
  public Optional<Double> lastKaValue1 = Optional.empty();
  public Optional<Double> lastKsValue1 = Optional.empty();
  public Optional<Double> lastKvValue2 = Optional.empty();
  public Optional<Double> lastKaValue2 = Optional.empty();
  public Optional<Double> lastKsValue2 = Optional.empty();
  public Slot0Configs slot0Configs;
  public Optional<Slot1Configs> slot1Configs;
  public Optional<Slot2Configs> slot2Configs;


  private SmartPIDControllerTalonFX(double kp, double ki, double kd, double kf, Optional<Double> kv, 
  Optional<Double> ka, Optional<Double> ks, String name, boolean smart, TalonFX motor) {
    this.motor = motor;
    this.name = name;
    this.smart = smart;

    slot0Configs = new Slot0Configs();

    slot0Configs.kP = kp;
    slot0Configs.kI = ki;
    slot0Configs.kD = kd;
    slot0Configs.kG = kf;

    if(kv.isPresent()) {
      putValueSmartDashboard(name, "Slot 0/kv Value", kv.get());
      slot0Configs.kV = kv.get();
      lastKvValue0 = kv;
    }

    if(ka.isPresent()) {
      putValueSmartDashboard(name, "Slot 0/ka Value", ka.get());
      slot0Configs.kA = ka.get();
      lastKaValue0 = ka;
    }

    if(ks.isPresent()) {
      putValueSmartDashboard(name, "Slot 0/ks Value", ks.get());
      slot0Configs.kS = ks.get();
      lastKsValue0 = ks;
    }

    motor.getConfigurator().apply(slot0Configs);

    putValueSmartDashboard(name, "Slot 0/kp Value", kp);
    putValueSmartDashboard(name, "Slot 0/ki Value", ki);
    putValueSmartDashboard(name, "Slot 0/kd Value", kd);
    putValueSmartDashboard(name, "Slot 0/kf Value", kf);
  }

  public SmartPIDControllerTalonFX(double kp, double ki, double kd, double kf, String name,
    boolean smart, TalonFX motor) {

    this(kp, ki, kd, kf, Optional.empty(), Optional.empty(), Optional.empty(), name, smart, motor);
  }

  public SmartPIDControllerTalonFX(double kp, double ki, double kd, double kf, double kv, double ka, double ks,  String name,
    boolean smart, TalonFX motor) {

    this(kp, ki, kd, kf, Optional.of(kv), Optional.of(ka), Optional.of(ks), name, smart, motor);
  }

  public void updatePID() {
    // if we pass this test, we are smart, so we can save some bandwith by only
    // grabing the k values once
    if (!smart || !Constants.Drivebase.PIDs.SMART_PID_ENABLED) {
      return;
    }

    updatePIDSlot0();

    if(slot1Configs.isPresent()) {
      updatePIDSlot1();
    }

    if(slot2Configs.isPresent()) {
      updatePIDSlot2();
    }

    putValueSmartDashboard(name, "Error", motor.getClosedLoopError().getValueAsDouble());
  }

  private void updatePIDSlot0() {
    double currentKpValue = getValueFromSmartDashboard(name, "Slot 0/kp Value", slot0Configs.kP);
    double currentKiValue = getValueFromSmartDashboard(name, "Slot 0/ki Value", slot0Configs.kI);
    double currentKdValue = getValueFromSmartDashboard(name, "Slot 0/kd Value", slot0Configs.kD);
    double currentKfValue = getValueFromSmartDashboard(name, "Slot 0/kf Value", slot0Configs.kG);

    boolean newPIDValues = false;

    Optional<Double> currentKv = Optional.empty();
    if(lastKvValue0.isPresent()) {
      currentKv = Optional.of(getValueFromSmartDashboard(name, "Slot 0/kv Value", lastKvValue0.get()));
      if(!currentKv.equals(lastKvValue0)) {
        lastKvValue0 = currentKv;
        slot0Configs.kV = currentKv.get();
        newPIDValues = true;
      }
    }

    Optional<Double> currentKa = Optional.empty();
    if(lastKaValue0.isPresent()) {
      currentKa = Optional.of(getValueFromSmartDashboard(name, "Slot 0/ka Value", lastKaValue0.get()));
      if(!currentKa.equals(lastKaValue0)) {
        lastKaValue0 = currentKa;
        slot0Configs.kA = currentKa.get();
        newPIDValues = true;
      }
    }

    Optional<Double> currentKs = Optional.empty();
    if(lastKsValue0.isPresent()) {
      currentKs = Optional.of(getValueFromSmartDashboard(name, "Slot 0/ks Value", lastKsValue0.get()));
      if(!currentKs.equals(lastKsValue0)) {
        lastKsValue0 = currentKs;
        slot0Configs.kS = currentKs.get();
        newPIDValues = true;
      }
    }

    if (currentKpValue != slot0Configs.kP || currentKiValue != slot0Configs.kI
        || currentKdValue != slot0Configs.kD || currentKfValue != slot0Configs.kG) {

      slot0Configs.kP = currentKpValue;
      slot0Configs.kI = currentKiValue;
      slot0Configs.kD = currentKdValue;
      slot0Configs.kG = currentKfValue;
      newPIDValues = true;
    }

    if(!newPIDValues) {
      return;
    }

    motor.getConfigurator().apply(slot0Configs);
  }

  private void updatePIDSlot1() {
    double currentKpValue = getValueFromSmartDashboard(name, "Slot 1/kp Value", slot1Configs.get().kP);
    double currentKiValue = getValueFromSmartDashboard(name, "Slot 1/ki Value", slot1Configs.get().kI);
    double currentKdValue = getValueFromSmartDashboard(name, "Slot 1/kd Value", slot1Configs.get().kD);
    double currentKfValue = getValueFromSmartDashboard(name, "Slot 1/kf Value", slot1Configs.get().kG);

    boolean newPIDValues = false;

    Optional<Double> currentKv = Optional.empty();
    if(lastKvValue1.isPresent()) {
      currentKv = Optional.of(getValueFromSmartDashboard(name, "Slot 1/kv Value", lastKvValue1.get()));
      if(!currentKv.equals(lastKvValue1)) {
        lastKvValue1 = currentKv;
        slot1Configs.get().kV = currentKv.get();
        newPIDValues = true;
      }
    }

    Optional<Double> currentKa = Optional.empty();
    if(lastKaValue1.isPresent()) {
      currentKa = Optional.of(getValueFromSmartDashboard(name, "Slot 1/ka Value", lastKaValue1.get()));
      if(!currentKa.equals(lastKaValue1)) {
        lastKaValue1 = currentKa;
        slot1Configs.get().kA = currentKa.get();
        newPIDValues = true;
      }
    }

    Optional<Double> currentKs = Optional.empty();
    if(lastKsValue1.isPresent()) {
      currentKs = Optional.of(getValueFromSmartDashboard(name, "Slot 1/ks Value", lastKsValue1.get()));
      if(!currentKs.equals(lastKsValue1)) {
        lastKsValue1 = currentKs;
        slot1Configs.get().kS = currentKs.get();
        newPIDValues = true;
      }
    }

    if (currentKpValue != slot1Configs.get().kP || currentKiValue != slot1Configs.get().kI
        || currentKdValue != slot1Configs.get().kD || currentKfValue != slot1Configs.get().kG) {

      slot1Configs.get().kP = currentKpValue;
      slot1Configs.get().kI = currentKiValue;
      slot1Configs.get().kD = currentKdValue;
      slot1Configs.get().kG = currentKfValue;
      newPIDValues = true;
    }

    if(!newPIDValues) {
      return;
    }

    motor.getConfigurator().apply(slot1Configs.get());
  }

  private void updatePIDSlot2() {
    double currentKpValue = getValueFromSmartDashboard(name, "Slot 2/kp Value", slot2Configs.get().kP);
    double currentKiValue = getValueFromSmartDashboard(name, "Slot 2/ki Value", slot2Configs.get().kI);
    double currentKdValue = getValueFromSmartDashboard(name, "Slot 2/kd Value", slot2Configs.get().kD);
    double currentKfValue = getValueFromSmartDashboard(name, "Slot 2/kf Value", slot2Configs.get().kG);

    boolean newPIDValues = false;

    Optional<Double> currentKv = Optional.empty();
    if(lastKvValue2.isPresent()) {
      currentKv = Optional.of(getValueFromSmartDashboard(name, "Slot 2/kv Value", lastKvValue2.get()));
      if(!currentKv.equals(lastKvValue2)) {
        lastKvValue2 = currentKv;
        slot2Configs.get().kV = currentKv.get();
        newPIDValues = true;
      }
    }

    Optional<Double> currentKa = Optional.empty();
    if(lastKaValue2.isPresent()) {
      currentKa = Optional.of(getValueFromSmartDashboard(name, "Slot 2/ka Value", lastKaValue2.get()));
      if(!currentKa.equals(lastKaValue2)) {
        lastKaValue2 = currentKa;
        slot2Configs.get().kA = currentKa.get();
        newPIDValues = true;
      }
    }

    Optional<Double> currentKs = Optional.empty();
    if(lastKsValue2.isPresent()) {
      currentKs = Optional.of(getValueFromSmartDashboard(name, "Slot 2/ks Value", lastKsValue2.get()));
      if(!currentKs.equals(lastKsValue2)) {
        lastKsValue2 = currentKs;
        slot2Configs.get().kS = currentKs.get();
        newPIDValues = true;
      }
    }

    if (currentKpValue != slot2Configs.get().kP || currentKiValue != slot2Configs.get().kI
        || currentKdValue != slot2Configs.get().kD || currentKfValue != slot2Configs.get().kG) {

      slot2Configs.get().kP = currentKpValue;
      slot2Configs.get().kI = currentKiValue;
      slot2Configs.get().kD = currentKdValue;
      slot2Configs.get().kG = currentKfValue;
      newPIDValues = true;
    }

    if(!newPIDValues) {
      return;
    }

    motor.getConfigurator().apply(slot2Configs.get());
  }

  private void AddSlot1Configs(double kp, double ki, double kd, double kf, Optional<Double> kv, 
  Optional<Double> ka, Optional<Double> ks) {

    slot1Configs = Optional.of(new Slot1Configs());

    slot1Configs.get().kP = kp;
    slot1Configs.get().kI = ki;
    slot1Configs.get().kD = kd;
    slot1Configs.get().kG = kf;

    if(kv.isPresent()) {
      putValueSmartDashboard(name, "Slot 1/kv Value", kv.get());
      slot1Configs.get().kV = kv.get();
      lastKvValue1 = kv;
    }

    if(ka.isPresent()) {
      putValueSmartDashboard(name, "Slot 1/ka Value", ka.get());
      slot1Configs.get().kA = ka.get();
      lastKaValue1 = ka;
    }

    if(ks.isPresent()) {
      putValueSmartDashboard(name, "Slot 1/ks Value", ks.get());
      slot1Configs.get().kS = ks.get();
      lastKsValue1 = ks;
    }

    motor.getConfigurator().apply(slot1Configs.get());

    putValueSmartDashboard(name, "Slot 1/kp Value", kp);
    putValueSmartDashboard(name, "Slot 1/ki Value", ki);
    putValueSmartDashboard(name, "Slot 1/kd Value", kd);
    putValueSmartDashboard(name, "Slot 1/kf Value", kf);
  }

  private void AddSlot2Configs(double kp, double ki, double kd, double kf, Optional<Double> kv, 
  Optional<Double> ka, Optional<Double> ks) {

    slot2Configs = Optional.of(new Slot2Configs());

    slot2Configs.get().kP = kp;
    slot2Configs.get().kI = ki;
    slot2Configs.get().kD = kd;
    slot2Configs.get().kG = kf;

    if(kv.isPresent()) {
      putValueSmartDashboard(name, "Slot 2/kv Value", kv.get());
      slot2Configs.get().kV = kv.get();
      lastKvValue2 = kv;
    }

    if(ka.isPresent()) {
      putValueSmartDashboard(name, "Slot 2/ka Value", ka.get());
      slot2Configs.get().kA = ka.get();
      lastKaValue2 = ka;
    }

    if(ks.isPresent()) {
      putValueSmartDashboard(name, "Slot 2/ks Value", ks.get());
      slot2Configs.get().kS = ks.get();
      lastKsValue2 = ks;
    }

    motor.getConfigurator().apply(slot2Configs.get());

    putValueSmartDashboard(name, "Slot 2/kp Value", kp);
    putValueSmartDashboard(name, "Slot 2/ki Value", ki);
    putValueSmartDashboard(name, "Slot 2/kd Value", kd);
    putValueSmartDashboard(name, "Slot 2/kf Value", kf);
  }

  public void AddSlot1Configs(double kp, double ki, double kd, double kf) {

    if(slot1Configs.isEmpty()) {
      AddSlot1Configs(kp, ki, kd, kf, Optional.empty(), Optional.empty(), Optional.empty());
    }
  }

  public void AddSlot1Configs(double kp, double ki, double kd, double kf, double kv, double ka, double ks) {

    if(slot1Configs.isEmpty()) {
      AddSlot1Configs(kp, ki, kd, kf, Optional.of(kv), Optional.of(ka), Optional.of(ks));
    }
  }

  public void AddSlot2Configs(double kp, double ki, double kd, double kf) {

    if(slot2Configs.isEmpty()) {
      AddSlot2Configs(kp, ki, kd, kf, Optional.empty(), Optional.empty(), Optional.empty());
    }
  }

  public void AddSlot2Configs(double kp, double ki, double kd, double kf, double kv, double ka, double ks) {

    if(slot2Configs.isEmpty()) {
      AddSlot2Configs(kp, ki, kd, kf, Optional.of(kv), Optional.of(ka), Optional.of(ks));
    }
  }
}
