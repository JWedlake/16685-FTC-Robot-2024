package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="DriveCode")
    public class DriveCode extends LinearOpMode {
        //Motor Construction
        DcMotor leftFront = null;
        DcMotor leftRear = null;
        DcMotor rightFront = null;
        DcMotor rightRear = null;
        DcMotorEx BoatPoleAngle = null;
        DcMotor BoatPoleExtension = null;
        DcMotor hangRight = null;
        DcMotor hangLeft = null;
        Servo servo1 = null;
        Servo wrist = null;
        //Boat Pole Variables
        double BPinputExtension = 0;
        double boatPoleAngle = 0;
        double boatPoleInputAngle = 0;
        int boatPoleAngleCast = 0;
        double wristPosition = .01;
        int hangAngle = 0;

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
        boolean fineAjustment = false;

        public void runOpMode() {
            //Initialize the Drive System
            {
                //Hardware Map the Motors
                rightFront = hardwareMap.get(DcMotor.class, "rightFront");
                rightRear = hardwareMap.get(DcMotor.class, "rightRear");
                leftFront = hardwareMap.get(DcMotor.class, "leftFront");
                leftRear = hardwareMap.get(DcMotor.class, "leftRear");
                BoatPoleAngle = (DcMotorEx) hardwareMap.get(DcMotor.class, "boatAngle");
                BoatPoleExtension = hardwareMap.get(DcMotor.class, "boatPull");
                hangRight = hardwareMap.get(DcMotor.class, "hangRight");
                hangLeft = hardwareMap.get(DcMotor.class, "hangLeft");
                servo1 = hardwareMap.get(Servo.class, "servo1");
                wrist = hardwareMap.get(Servo.class, "wrist");

                //Set the Direction of Rotation for Each Motor
                rightFront.setDirection(DcMotor.Direction.REVERSE);
                rightRear.setDirection(DcMotor.Direction.REVERSE);
                leftFront.setDirection(DcMotor.Direction.FORWARD);
                leftRear.setDirection(DcMotor.Direction.FORWARD);
                BoatPoleAngle.setDirection(DcMotor.Direction.FORWARD);
                BoatPoleExtension.setDirection(DcMotor.Direction.FORWARD);
                hangRight.setDirection(DcMotor.Direction.FORWARD);
                hangLeft.setDirection(DcMotor.Direction.REVERSE);

                servo1.setPosition(0.5);
                wrist.setPosition(wristPosition);
                BoatPoleAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BoatPoleAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                boatPoleInputAngle = -75;
                boatPoleAngle = 0;
                BoatPoleExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BoatPoleExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BPextension(0,1);
                hangAngle = 0;
                hangRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hangRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hangLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hangLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
                    if (gamepad2.left_stick_button){
                        fineAjustment=true;
                    }
                    if (gamepad2.right_stick_button){
                        fineAjustment=false;
                    }

                    if (gamepad2.right_trigger > 0 && BPinputExtension < 2400) {
                            BPinputExtension += 10;
                            BPinputExtension = Math.min(Math.max(BPinputExtension, -50), 2400);
                            BPextension((int) BPinputExtension, .5);
                        }
                    if (gamepad2.left_trigger > 0) {
                        BPinputExtension -= 10;
                        BPinputExtension = Math.min(Math.max(BPinputExtension, -100), 3750);
                        BPextension((int) BPinputExtension, .5);
                    }
                    if (gamepad2.left_stick_y < 0 && boatPoleAngle < 730) {
                        boatPoleAngle += 3;
                        boatPoleAngle = Math.min(Math.max(boatPoleAngle, 0), 730);
                        BPAngle((int) boatPoleAngle, .5);
                    }
                    if (gamepad2.left_stick_y > 0) {

                        if(boatPoleAngle<200||fineAjustment) {
                            boatPoleAngle -= 1;
                        }
                        else {
                            boatPoleAngle -= 3;
                        }
                        boatPoleAngle = Math.min(Math.max(boatPoleAngle, 0), 730);
                        BPAngle((int) boatPoleAngle, .5);
                    }

                    if (gamepad2.right_stick_y > 0){
                        wristPosition -= .005;
                        wristPosition = Math.min(Math.max(wristPosition, 0), 1);
                        wrist.setPosition(wristPosition);
                    }
                    if (gamepad2.right_stick_y < 0){
                        wristPosition += .005;
                        wristPosition = Math.min(Math.max(wristPosition, 0), 1);
                        wrist.setPosition(wristPosition);
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
                    /*if (gamepad2.right_bumper) {
                        hangLeft.setPower(-hangAngle);
                        hangRight.setPower(-hangAngle);
                        sleep(500);
                        hangLeft.setPower(0);
                        hangRight.setPower(0);
                    }*/
//                    if((1*hangRight.getCurrentPosition())<hangAngle){
//                        hangRight.setPower(1);
//                    }
//                    if((1*hangRight.getCurrentPosition())>=hangAngle) {
//                        hangRight.setPower(0);
//                    }
                    {
                        telemetry.addData("Status", "Driver Control");
                        telemetry.addData("PowerMultipler", "Value (%.2f)", drivePower);
                        telemetry.addData("BoatPoleAngle", ((int) ((BoatPoleAngle.getCurrentPosition() / ((537.7 * 4.775) / 360) - 76))));
                        telemetry.addData("BoatPoleCast", boatPoleAngleCast);
                        telemetry.addData("BoatPoleAngle", ((int) boatPoleAngle));
                        telemetry.addData("LeftHang", hangLeft.getCurrentPosition());
                        telemetry.addData("RightHang", hangRight.getCurrentPosition());
                        telemetry.update();
                    }
                    //End of while statement
                }
                //End of while statement
            }
//end of opModeIsActive loop
        }

        private void BPextension(int ticks,double power) {
            //ticks = Math.min(Math.max(ticks,0), 3000);
            BoatPoleExtension.setTargetPosition(ticks);
            BoatPoleExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BoatPoleExtension.setPower(power);
        }
        private void BPAngle(int ticks,double power) {
            //ticks = Math.min(Math.max(ticks,0), 3000);
            BoatPoleAngle.setTargetPosition(ticks);
            BoatPoleAngle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BoatPoleAngle.setPower(power);

        }
        private void HangAngle(int ticks, double power) {

            hangLeft.setTargetPosition(ticks);
            hangLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hangLeft.setPower(power);
            hangRight.setTargetPosition(ticks);
            hangRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hangRight.setPower(power);
            }

        }

