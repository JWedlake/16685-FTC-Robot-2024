package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
import java.util.concurrent.TimeUnit;

    @TeleOp(name="DriveCode")
    public class DriveCode extends LinearOpMode {
        //Motor Construction
        DcMotor leftFront = null;
        DcMotor leftRear = null;
        DcMotor rightFront = null;
        DcMotor rightRear = null;

        //Servo Construction
        //Servo (name) = null;

        //Drive Variables
        float turn = 0;
        double drivePower = 0.0;
        double MLF = 0;
        double MLR = 0;
        double MRF = 0;
        double MRR = 0;
        double x = 0;
        double y = 0;
        double theta = 0;
        double power = 0;
        double sin = 0;
        double cos = 0;
        double max = 0.0;

        public void runOpMode() {
            //Initialize the Drive System
            {
                //Hardware Map the Motors
                rightFront = hardwareMap.get(DcMotor.class, "rightFront");
                rightRear = hardwareMap.get(DcMotor.class, "rightRear");
                leftFront = hardwareMap.get(DcMotor.class, "leftFront");
                leftRear = hardwareMap.get(DcMotor.class, "leftRear");

                //Set the Direction of Rotation for Each Motor
                rightFront.setDirection(DcMotor.Direction.FORWARD);
                rightRear.setDirection(DcMotor.Direction.FORWARD);
                leftFront.setDirection(DcMotor.Direction.REVERSE);
                leftRear.setDirection(DcMotor.Direction.REVERSE);

                //Tell the User that the Drive System is Initialized
                telemetry.addData("Status", "Drive System Initialized");
                telemetry.addData("Drive Power", "Power (%.2f)", drivePower);
            }

//Wait for Start
            waitForStart();

            if (opModeIsActive()) {
                while (opModeIsActive()) {

                    // TeleOP Drive Control
                    {
                        //Converts Driver Inputs into Power and Angle
                        x = (gamepad1.left_stick_x * -1);
                        y = gamepad1.left_stick_y;
                        turn = gamepad1.right_stick_x;
                        theta = Math.atan2(y, x);
                        power = Math.hypot(x, y);

                        //Calculates Each Motor's Power using the Direction and Magnitude of the Controller
                        sin = Math.sin(theta - Math.PI / 4);
                        cos = Math.cos(theta - Math.PI / 4);
                        max = Math.max(Math.abs(sin), Math.abs(cos));

                        MLF = (power * cos / max + turn);
                        MRF = (power * sin / max - turn);
                        MLR = (power * sin / max + turn);
                        MRR = (power * cos / max - turn);
                        //If Any of the Motors Output is Greater than Full Power, Ajust the Motor Power to Keep the Ratios Between the Motor the Same.
                        if (((power + Math.abs(turn))*drivePower) > 1) {
                            MLF = MLF / (power + Math.abs(turn));
                            MRF = MRF / (power + Math.abs(turn));
                            MLR = MLR / (power + Math.abs(turn));
                            MRR = MRR / (power + Math.abs(turn));
                        }

                        //Cast the Drive Motors Power Value to the Motors
                        rightFront.setPower(MRF * drivePower);
                        leftFront.setPower(MLF * drivePower);
                        rightRear.setPower(MRR * drivePower);
                        leftRear.setPower(MLR * drivePower);

                        //End of Drive Control
                    }

                    //Drive Power Ajustments
                    if (gamepad1.right_bumper){
                        drivePower = drivePower = 1;
                    }
                    if (gamepad1.right_trigger >= .2){
                        drivePower = drivePower = 0.63;
                    }
                    if (gamepad1.left_trigger >= .2) {
                        drivePower = drivePower = 0.25;
                    }

                    //Drive Control Telemetry Cast
                    {
                        telemetry.addData("Status", "Driver Control");
                        telemetry.addData("PowerMultipler", "Value (%.2f)", drivePower);
                        telemetry.update();
                    }
                    //End of while statement
                }
//end of opModeIsActive loop
            }
        }
    }
