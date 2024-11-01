package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.BoatPole;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;

public class BoatPoleCode extends DriveCode {

    //Motor Construction
    DcMotorEx BoatPoleAngle = null; // Motor that Ajusts the Angle of the Boat Pole
    DcMotor BoatPoleExtension = null; // Motor that Drives the Boat Pole
    BoatPole Intake = new BoatPole(BoatPoleAngle,BoatPoleExtension); // BoatPole Object Creation
    String initPhase = "IntakeBPAngle"; // String for State Based Logic

    public void runOpMode() {
        //Initialize the Drive System
        {
            //Hardware Map the Motors
            BoatPoleAngle = (DcMotorEx) hardwareMap.get(DcMotor.class, "boatPole");
            BoatPoleExtension = hardwareMap.get(DcMotor.class, "boatAngle");

            //Set the Direction of Rotation for Each Motor
            BoatPoleAngle.setDirection(DcMotor.Direction.FORWARD);
            BoatPoleExtension.setDirection(DcMotor.Direction.FORWARD);

            Intake.Intialization(initPhase);
/*
            // Start Intitalization Here

            if (initPhase.equals("IntakeBPAngle")) {

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
*/
                //Tell the User that the Boat Pole System is Initialized

                telemetry.addData("Status", "BoatPole Initialized");
            }

//Wait for Start
            waitForStart();

            if (opModeIsActive()) {
                while (opModeIsActive()) {
                    //End of while statement
                }
//end of opModeIsActive loop
            }
        }

    }


