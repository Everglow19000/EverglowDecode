package org.firstinspires.ftc.teamcode.everglow_library;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.SwerveModuleState;

public class SwerveModule {
    private MotorEx motor;
    private CRServoEx servo;
    private double maxSpeed;
    public SwerveModule(HardwareMap hardwareMap, String motorName, boolean isMotorFlipped, String servoName, boolean isServoFlipped, String servoEncoderName, double wheelDiameter, double motorCPR, double motorRPM) {
        motor = new MotorEx(hardwareMap, motorName, motorCPR, motorRPM);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setInverted(isMotorFlipped);
        servo = new CRServoEx(hardwareMap, servoName, new AbsoluteAnalogEncoder(hardwareMap, servoEncoderName).setReversed(isServoFlipped), CRServoEx.RunMode.OptimizedPositionalControl);
        servo.setInverted(isServoFlipped);
        maxSpeed = (motor.getMaxRPM()/60)*Math.PI*wheelDiameter;
    }
    public SwerveModule(HardwareMap hardwareMap, String motorName, boolean isMotorFlipped, String servoName, boolean isServoFlipped, String servoEncoderName, double wheelDiameter, Motor.GoBILDA motorType) {
        this(hardwareMap, motorName, isMotorFlipped, servoName, isServoFlipped, servoEncoderName, wheelDiameter, motorType.getCPR(), motorType.getRPM());
    }

    public void setState(SwerveModuleState state) {
        motor.set(state.speedMetersPerSecond/maxSpeed);
        servo.set(state.angle.getRadians());
    }

    public void set(double power, double angle) {
        motor.set(power);
        servo.set(angle);
    }

    public void stopMotor() {
        motor.set(0);
    }
}
