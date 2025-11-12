package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="CloseAutonomous_blue")

public class CloseAutonomous_blue extends CloseAutonomous{
    @Override
    public void runOpMode() throws InterruptedException {
        isBlue = true;
        super.runOpMode();
}}
