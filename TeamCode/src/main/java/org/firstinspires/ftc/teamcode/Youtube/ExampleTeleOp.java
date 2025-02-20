package org.firstinspires.ftc.teamcode.Youtube;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Configuration File
 * Control Hub:
 * Port 01: leftFront
 *
 * Expansion Hub:
 *
 */
@Disabled
@TeleOp(group = "Primary")
public class ExampleTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        //Initialization Phase/just run once
        initHardware();
        //Loop before the run button is pressed
        while(!isStarted()) {}
        //things you want to happen once before the program loops
        waitForStart();
        //run while the code is active
        while(opModeIsActive()){}

    }

    public void initHardware() {

    }
}

