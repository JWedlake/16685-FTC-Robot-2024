package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BoatPole extends BoatPoleCode {

    //Object Variables
    double maxAngle = 115; // Angle from vertical
    double minAngle = 112.5; // Angle from min angle reaching down
    double angleGearRatio = 5; // Gear Ratio Between Motor and Angle Arm
    double spoolGearRatio = 2.5; // Gear Ratio Between Motor and Spool
    double spoolDiameter = 3; // Diameter of the Spool to Calculate the Extension
    public BoatPole(DcMotorEx BoatPoleAngle, DcMotor BoatPoleExtension) {
        // BoatPole construction without declaring variables
    }
    public BoatPole(double MaxAngle, double MinAngle, double AngleGearRatio, double SpoolGearRatio, double SpoolDiameter, DcMotorEx BoatPoleAngle, DcMotor BoatPoleExtension) {
        // BoatPole construction declaring each variable
        maxAngle = MaxAngle;
        minAngle = MinAngle;
        angleGearRatio = AngleGearRatio;
        spoolGearRatio = SpoolGearRatio;
        spoolDiameter = SpoolDiameter;
    }

    public void Intialization(String initPhase) {
        BoatPoleAngle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BoatPoleAngle.setPower(.05);
        while (initPhase.equals("IntakeBPAngle")) {
            if (BoatPoleAngle.getVelocity() <= .1) {
                BoatPoleAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BoatPoleAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BoatPoleAngle.setPower(.5);
                break;
            }
        }

    }
}

