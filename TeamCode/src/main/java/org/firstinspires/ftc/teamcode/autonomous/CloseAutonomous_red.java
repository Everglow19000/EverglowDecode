package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="CloseAutonomous_red")

public class CloseAutonomous_red extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CloseAutonomous autonomous = new CloseAutonomous(this, false);
        autonomous.run();
    }
}
