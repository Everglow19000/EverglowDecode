package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="CloseAutonomous_red")

public class CloseAutonomous_red extends CloseAutonomous{
    @Override
    public void runOpMode() throws InterruptedException {
        isBlue = false;
        super.runOpMode();
    }}
