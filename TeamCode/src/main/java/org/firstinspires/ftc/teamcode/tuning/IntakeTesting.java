package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name="IntakeTesting", group="Tests")
@Config
public class IntakeTesting extends LinearOpMode {
    public static double powerToUse = 0.1;
    public static boolean isWork = false;
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {
        Intake.INTAKE_POWER = powerToUse;
        intake = new Intake(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            Intake.INTAKE_POWER = powerToUse;
            if (isWork) {
                intake.startIntake();
            }
            else {
                intake.stopIntake();
            }
        }
    }
}
