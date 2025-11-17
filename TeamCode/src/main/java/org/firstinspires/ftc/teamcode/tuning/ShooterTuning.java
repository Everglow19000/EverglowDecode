package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "ShooterTuningFormula")
@Config
public class ShooterTuning extends LinearOpMode {
    public static double tickPerSecond = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Shooter shooter = new Shooter(hardwareMap);


        while (opModeIsActive()) {
            shooter.setFlywheelMotorSpeed(tickPerSecond);
            shooter.update(0);

            telemetry.addData("intended speed", tickPerSecond);
            telemetry.addData("recorded speed", shooter.getFlywheelMotorCurrentTicksPerSecond());
            telemetry.addData("recorded speed RPM", shooter.getFlywheelMotorCurrentRPM());
            telemetry.update();
        }
    }
}