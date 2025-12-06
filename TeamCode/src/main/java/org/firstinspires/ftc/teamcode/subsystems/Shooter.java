package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.everglow_library.Subsystem;
import org.firstinspires.ftc.teamcode.everglow_library.Utils;

public class Shooter implements Subsystem {

    // --------------------
    // | Servo Parameters |
    // --------------------
    public static double minServoPosition = 0.275;
    public static double maxServoPosition = 0.48;
    public static double minServoAngle = 3.7;
    public static double maxServoAngle = 44.3;
    private static Servo.Direction servoDirection = Servo.Direction.REVERSE;
    private static InterpLUT servoAnglesLUT = Utils.interpLUTFromArrays( //TODO: FILL ME WITH MEASURED VALUES
            new double[]{
                    15.1,
                    78.8,
                    126.7,
                    152.2
            },
            new double[]{
                    10.8,
                    36.18,
                    36.18,
                    36.18
            }
    );


    // -----------------------
    // | Flywheel Parameters |
    // -----------------------
    private boolean flywheelShouldSpin = false;
    private static boolean isFlywheelInverted = true;
    private static InterpLUT flywheelSpeedsLUT = Utils.interpLUTFromArrays( //TODO: FILL ME WITH MEASURED VALUES
            new double[]{
                    15.1,
                    78.8,
                    126.7,
                    152.2
            },
            new double[]{
                    1200,
                    1400,
                    1700,
                    1800
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
            flywheelMotor.set(flywheelPIDF.calculate(getFlywheelMotorCurrentTicksPerSecond(), desiredFlywheelSpeed));

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
            setHoodServoPosition(Math.min(targetServoPosition + step, targetPosition));

            return targetServoPosition != targetPosition;
        }
    }


    MotorEx flywheelMotor;
    Servo hoodServo;
    public PIDFController flywheelPIDF = new PIDFController(0.025, 0.2, 0, 0.0001);
    private double desiredFlywheelSpeed = 0; // [ticks/s]
    private double targetServoPosition = 0;


    public Shooter(HardwareMap hardwareMap) {
        flywheelMotor = new MotorEx(hardwareMap, "flywheelMotor", Motor.GoBILDA.BARE);

        flywheelMotor.setInverted(isFlywheelInverted);
        flywheelMotor.setRunMode(Motor.RunMode.RawPower);

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        Utils.setServoPWMRange(hoodServo, 500, 2500);
        hoodServo.setDirection(servoDirection);
    }

    public double getFlywheelTicksPerSecondForDistanceFromGoal(double distance) {
        return flywheelSpeedsLUT.get(clampDistance(distance));
    }
    public double getServoAngleForDistanceFromGoal(double distance) {
        return servoAnglesLUT.get(clampDistance(distance));
    }


    public void setFlywheelMotorSpeed(double ticksPerSecond) {
        if (ticksPerSecond == 0) {
            stopMotor();
            return;
        }
        flywheelShouldSpin = true;
        desiredFlywheelSpeed = ticksPerSecond;
    }

    public double hoodDegreesToServoPosition(double hoodDegrees) {
        return ((hoodDegrees-minServoAngle)/(maxServoAngle-minServoAngle));
    }
    public double servoPositionToHoodDegrees(double position) {
        return minServoAngle + (position*(maxServoAngle-minServoAngle));
    }

    public double getFlywheelMotorCurrentRPM() {
        return (flywheelMotor.getCorrectedVelocity()/flywheelMotor.getCPR())*60.0;
    }

    public double getFlywheelPower() {
        return flywheelMotor.get();
    }

    public double getFlywheelMotorCurrentTicksPerSecond() {
        return flywheelMotor.getCorrectedVelocity();
    }

    private double clampDistance(double distance) {
        return Math.max(15.1+0.1, Math.min(152.2-0.1, distance));
    }

    public void setHoodServoAngle(double angle) {
        setHoodServoPosition(hoodDegreesToServoPosition(angle));
    }

    public void setHoodServoPosition(double pos) {
        pos = minServoPosition + ((maxServoPosition-minServoPosition)*pos);
        targetServoPosition = pos;
        hoodServo.setPosition(pos);
    }

    public double getHoodServoPosition() {
        return (targetServoPosition - minServoPosition)/(maxServoPosition-minServoPosition);
    }

    public void stopMotor() {
        flywheelMotor.stopMotor();
        flywheelShouldSpin = false;
        desiredFlywheelSpeed = 0;
    }

    public boolean isFlywheelFinishedSpinning() {
        return Math.abs(flywheelMotor.getCorrectedVelocity() - desiredFlywheelSpeed) <= 50;
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
        if (flywheelShouldSpin) {
            flywheelMotor.set(
                    flywheelPIDF.calculate(
                            getFlywheelMotorCurrentTicksPerSecond(),
                            desiredFlywheelSpeed
                    )
            );
        }
    }

    @Override
    public String status() {
        return "";
    }
}
