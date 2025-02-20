package org.firstinspires.ftc.teamcode.drive;
import org.firstinspires.ftc.teamcode.drive.MotorExtended;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name="LinearActuator")
public class LinearActuator extends LinearOpMode {
    //Initialization Varriables

        //For all of these it allows you to change the port name all in one spot and fix the configuration

        //Hang Motor
        MotorExtended hangMotor = new MotorExtended(true,true,1,"hangRight",true);
        //hangMotor.setEncoder(true);
        MotorExtended linearActuator = new MotorExtended();
        //linearActuator.setEncoder(true);

        //DriveMotors
        DcMotor leftFront = null;
        DcMotor leftRear = null;
        DcMotor rightFront = null;
        DcMotor rightRear = null;

        //Motor Construction
        DcMotor hangRight = null;
        DcMotor LinearActuator = null;
        Servo servo1 = null;
        //Boat Pole Variables
        int hangAngle = 0;
        int linearHeight = 0;
        private VoltageSensor voltageSensor;


        //Drive Variables
        float turn = 0;
        double drivePower = 0.5;
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
        double presentVoltage;

        public void runOpMode() {
            //Initialize the Drive System
            {
                //Hardware Map the Motors
                rightFront = hardwareMap.get(DcMotor.class, "rightFront");
                rightRear = hardwareMap.get(DcMotor.class, "rightRear");
                leftFront = hardwareMap.get(DcMotor.class, "leftFront");
                leftRear = hardwareMap.get(DcMotor.class, "leftRear");
                hangRight = hardwareMap.get(DcMotor.class, "hangRight");
                servo1 = hardwareMap.get(Servo.class, "servo1");
                voltageSensor = hardwareMap.get(VoltageSensor.class,"Control Hub");

                //Set the Direction of Rotation for Each Motor
                rightFront.setDirection(DcMotor.Direction.REVERSE);
                rightRear.setDirection(DcMotor.Direction.REVERSE);
                leftFront.setDirection(DcMotor.Direction.FORWARD);
                leftRear.setDirection(DcMotor.Direction.FORWARD);
                hangRight.setDirection(DcMotor.Direction.FORWARD);

                servo1.setPosition(0.5);
                hangAngle = 0;
                hangRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hangRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
                        turn = -gamepad1.right_stick_x;
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
                        if (((power + Math.abs(turn)) * drivePower) > 1) {
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
                    if (gamepad1.right_bumper) {
                        drivePower = drivePower = 1;
                    }
                    if (gamepad1.right_trigger >= .2) {
                        drivePower = drivePower = 0.63;
                    }
                    if (gamepad1.left_trigger >= .2) {
                        drivePower = drivePower = 0.25;
                    }

                    presentVoltage = voltageSensor.getVoltage();
                    //Grabber
                    if (gamepad2.a) {
                        servo1.setPosition(0.48);
                    }
                    if (gamepad2.b) {
                        servo1.setPosition(0.5);
                    }
                    if (gamepad2.x) {
                        servo1.setPosition(0.35);
                    }
                    if (gamepad2.y) {
                        servo1.setPosition(.3);
                    }

                    if (gamepad2.right_trigger > 0 && linearHeight < 2400) {
                        linearHeight += 10;
                        linearHeight = Math.min(Math.max(linearHeight, -50), 2400);
                        linearActuator( linearHeight, 1);
                    }
                    if (gamepad2.left_trigger > 0) {
                        linearHeight -= 10;
                        linearHeight = Math.min(Math.max(linearHeight, 0), 2400);
                        linearActuator( linearHeight, 1);
                    }
                    if(gamepad2.right_bumper) {
                        linearHeight = 780;//IDK just an example varriable for the time being but allign to the wall
                        linearActuator( linearHeight, 1);
                    }

                    if(gamepad2.left_bumper) {
                        Hook();
                    }
                    //hang code
                    if (gamepad2.dpad_up) {
                        hangAngle += 20;
                        hangAngle = Math.min(Math.max(hangAngle, -2500), 0);
                        HangAngle(hangAngle,1);
                    }

                    if (gamepad2.dpad_down) {
                        hangAngle -= 20;
                        hangAngle = Math.min(Math.max(hangAngle, -2500), 0);
                        HangAngle(hangAngle,1);
                    }
                    presentVoltage = voltageSensor.getVoltage();
                    {
                        telemetry.addData("Status", "Driver Control");
                        telemetry.addData("PowerMultipler", "Value (%.2f)", drivePower);
                        telemetry.addData("RightHang", hangRight.getCurrentPosition());
                        telemetry.addData("LinearActuator Position ", LinearActuator.getCurrentPosition());
                        telemetry.update();
                    }
                    //End of while statement
                }
                //End of while statement
            }
//end of opModeIsActive loop
        }

        private void linearActuator(int ticks,double power) {
            //ticks = Math.min(Math.max(ticks,0), 3000);
            LinearActuator.setTargetPosition(ticks);
            LinearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LinearActuator.setPower(power);
        }

        private void Hook() {
            linearHeight=2000;//Need to determine top point
            linearActuator(linearHeight,1);
            while(LinearActuator.isBusy()) {
            }
            double voltageBenchmark = voltageSensor.getVoltage();
            linearHeight=1850;//Height to hook just barely stressing actuator
            linearActuator(linearHeight,1);
            while(LinearActuator.isBusy()) {
                if ((voltageBenchmark - voltageSensor.getVoltage()) > 2) //need to tune voltage amount
                {
                    servo1.setPosition(.3);
                }
            }
        }
        private void HangAngle(int ticks, double power) {
            hangRight.setTargetPosition(ticks);
            hangRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hangRight.setPower(power);
        }

    }



