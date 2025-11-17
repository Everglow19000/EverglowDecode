package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.everglow_library.Subsystem;

public class Shooter implements Subsystem {
    private static double gravity = 9.81;

    // --------------------
    // | Servo Parameters |
    // --------------------
    private static double minServoPosition = 0.39;
    private static double maxServoPosition = 0.63;
    private static double minServoAngle = 0.0698132;
    private static double maxServoAngle = 0.872665;
    private static Servo.Direction servoDirection = Servo.Direction.REVERSE;


    // -----------------------
    // | Flywheel Parameters |
    // -----------------------
    private static double goalHeight = 0.9843; // meters above the field of the lip of the goal
    private static double shooterHeight = 0.26; // meters above the field of artifact as it leaves shooter TODO: Change according to actual robot measurements
    private static boolean isFlywheelInverted = true;
    private static double flywheelRadius = 0.048; // in meters

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
        private boolean hasStarted = false;
        private double givenDistanceFromGoal;

        private StartUpShooterAction(double distanceFromGoal, double shootingAngle) {
            this.givenDistanceFromGoal = distanceFromGoal;
            desiredFlywheelSpeed = getFlywheelTicksPerSecondForArtifactVelocity(getArtifactVelocityForDistanceAndAngle(distanceFromGoal, shootingAngle));
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!hasStarted) {
                flywheelMotor.setVelocity(desiredFlywheelSpeed);

                hasStarted = true;
            }

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

    public static double getArtifactVelocityForDistanceAndAngle(double distance, double angle) {
        //stolen from internet so might not be good
        return Math.sqrt((gravity*Math.pow(distance, 2))/(2*Math.pow(Math.cos(angle), 2)*((distance*Math.tan(angle))-(goalHeight - shooterHeight))));
    }

    public double getFlywheelTicksPerSecondForArtifactVelocity(double artifactVelocity) {
        return ((artifactVelocity/flywheelRadius)/(2*Math.PI))*flywheelMotor.getCPR();
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

    public void setHoodServoAngle(double angle, AngleUnit unit) {
        double wantedAngle = unit.toRadians(angle);
        setHoodServoPosition((wantedAngle - minServoAngle)/(maxServoAngle - minServoAngle));
    }

    public void setHoodServoPosition(double pos) {
        targetServoPos = pos;
        hoodServo.setPosition(pos);
    }

    public double getHoodServoAngle() {
        return targetServoPos*(maxServoAngle-minServoAngle) + minServoAngle;
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

    public StartUpShooterAction getStartUpShooterAction(double distanceFromGoal, double shootingAngle) {
        return new StartUpShooterAction(distanceFromGoal, shootingAngle);
    }

    public AimHoodAction getAimHoodAction(double wantedPos) {
        return new AimHoodAction(wantedPos);
    }
    public AimHoodAction getAimHoodAction(double wantedAngle, AngleUnit unit) {
        return getAimHoodAction((unit.toRadians(wantedAngle)-minServoAngle)/(maxServoAngle - minServoAngle));
    }

    @Override
    public void update(int iterationCount) {
        flywheelMotor.set(
                flywheelPID.calculate(
                        flywheelMotor.getCorrectedVelocity(),
                        desiredFlywheelSpeed
                )
        );
    }

    @Override
    public String status() {
        return "";
    }
}
