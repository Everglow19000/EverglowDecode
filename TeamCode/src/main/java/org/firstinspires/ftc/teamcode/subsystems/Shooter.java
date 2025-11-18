package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.everglow_library.Subsystem;
import org.firstinspires.ftc.teamcode.everglow_library.Utils;

public class Shooter implements Subsystem {

    // --------------------
    // | Servo Parameters |
    // --------------------
    private static double minServoPosition = 0.39;
    private static double maxServoPosition = 0.63;
    private static Servo.Direction servoDirection = Servo.Direction.REVERSE;
    private static InterpLUT servoPositionsLUT = Utils.interpLUTFromArrays( //TODO: FILL ME WITH MEASURED VALUES
            new int[]{
                    0
            },
            new int[]{
                    0
            }
    );


    // -----------------------
    // | Flywheel Parameters |
    // -----------------------
    private static boolean isFlywheelInverted = true;
    private static InterpLUT flywheelSpeedsLUT = Utils.interpLUTFromArrays( //TODO: FILL ME WITH MEASURED VALUES
            new int[]{
                    0
            },
            new int[]{
                0
            }
    );

    public class StopShooterSpinAction implements Action {
        private StopShooterSpinAction() {

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            stopMotor();
            return false;
        }
    }

    public class StartUpShooterAction implements Action {
        private double givenDistanceFromGoal;

        private StartUpShooterAction(double distanceFromGoal) {
            this.givenDistanceFromGoal = distanceFromGoal;
            desiredFlywheelSpeed = getFlywheelTicksPerSecondForDistanceFromGoal(distanceFromGoal);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            flywheelMotor.set(flywheelPID.calculate(getFlywheelMotorCurrentTicksPerSecond(), desiredFlywheelSpeed));

            return !isFlywheelFinishedSpinning();
        }
    }

    public class AimHoodAction implements Action {
        private double targetPosition;
        private double step = 0.008;

        private AimHoodAction(double pos) {
            targetPosition = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setHoodServoPosition(Math.min(targetServoPos + step, targetPosition));

            return targetServoPos != targetPosition;
        }
    }


    MotorEx flywheelMotor;
    Servo hoodServo;
    PIDController flywheelPID = new PIDController(0.2, 0.01, 0.0);
    private double desiredFlywheelSpeed = 0; // [ticks/s]
    private double targetServoPos = 0;


    public Shooter(HardwareMap hardwareMap) {
        flywheelMotor = new MotorEx(hardwareMap, "flywheelMotor", Motor.GoBILDA.BARE);

        flywheelMotor.setInverted(isFlywheelInverted);
        flywheelMotor.setRunMode(Motor.RunMode.RawPower);


        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        hoodServo.setDirection(servoDirection);
        hoodServo.scaleRange(minServoPosition, maxServoPosition);
    }

    public double getFlywheelTicksPerSecondForDistanceFromGoal(double distance) {
        return flywheelSpeedsLUT.get(distance);
    }
    public double getServoPositionForDistanceFromGoal(double distance) {
        return servoPositionsLUT.get(distance);
    }


    public void setFlywheelMotorSpeed(double ticksPerSecond) {
        desiredFlywheelSpeed = ticksPerSecond;
    }

    public double getFlywheelMotorCurrentRPM() {
        return (flywheelMotor.getCorrectedVelocity()/flywheelMotor.getCPR())*60.0;
    }

    public double getFlywheelMotorCurrentTicksPerSecond() {
        return flywheelMotor.getCorrectedVelocity();
    }

    public void setHoodServoPosition(double pos) {
        targetServoPos = pos;
        hoodServo.setPosition(pos);
    }

    public double getHoodServoPosition() {
        return targetServoPos;
    }

    public void stopMotor() {
        flywheelMotor.stopMotor();
    }

    public boolean isFlywheelFinishedSpinning() {
        return flywheelMotor.getCorrectedVelocity() - desiredFlywheelSpeed <= 10;
    }

    public StopShooterSpinAction getStopShooterSpinAction() {
        return new StopShooterSpinAction();
    }

    public StartUpShooterAction getStartUpShooterAction(double distanceFromGoal) {
        return new StartUpShooterAction(distanceFromGoal);
    }

    public AimHoodAction getAimHoodAction(double wantedPos) {
        return new AimHoodAction(wantedPos);
    }

    @Override
    public void update(int iterationCount) {
        flywheelMotor.set(
                flywheelPID.calculate(
                        getFlywheelMotorCurrentTicksPerSecond(),
                        desiredFlywheelSpeed
                )
        );
    }

    @Override
    public String status() {
        return "";
    }
}
