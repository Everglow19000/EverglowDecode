package org.firstinspires.ftc.teamcode.subsystems;

import android.app.Notification;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.everglow_library.Subsystem;

public class Intake implements Subsystem {

    private class IntakeAction implements Action {
        private boolean isTakeIn;
        private double timeUntilConsideredDoneMilliseconds;
        private boolean hasStarted = false;
        private double startTime;
        public IntakeAction(boolean takeIn, double timeUntilConsideredDoneMilliseconds) {
            this.isTakeIn = takeIn;
            this.timeUntilConsideredDoneMilliseconds = timeUntilConsideredDoneMilliseconds;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!hasStarted) {
                if (isTakeIn) {
                    startIntake();
                }
                else {
                    stopIntake();
                }

                hasStarted = true;
                this.startTime = System.currentTimeMillis();
            }

            return System.currentTimeMillis() - startTime < timeUntilConsideredDoneMilliseconds;
        }
    }
    public static double INTAKE_POWER = 0.6;
    private DcMotorSimple intakeMotor;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorSimple.class, "IntakeMotor");
    }

    public void startIntake() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }

    public boolean isIntaking() {
        return intakeMotor.getPower() == INTAKE_POWER;
    }


    public IntakeAction getStartIntakeAction() {
        return new IntakeAction(true, 0);
    }

    public IntakeAction getStopIntakeAction() {
        return new IntakeAction(false, 0);
    }

    public Action getIntakeActionForAmountOfTime(double timeAmountMillis) {
        return new SequentialAction(
                new IntakeAction(true, timeAmountMillis),
                new IntakeAction(false, 0)
        );
    }

    @Override
    public void update(int iterationCount) {
        // nothing to do here yet.
    }

    @Override
    public String status() {
        return "IsIntaking: " + isIntaking();
    }
}
