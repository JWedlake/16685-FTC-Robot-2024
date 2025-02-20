package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="messyAuton")
public class messyAuton extends LinearOpMode {
    @Override
    public void runOpMode() {
        /*SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .forward(5)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
    }*/

        DcMotor leftFront = null;
        DcMotor leftRear = null;
        DcMotor rightFront = null;
        DcMotor rightRear = null;
        DcMotor BoatPoleAngle = null;
        DcMotor hangRight = null;
        DcMotor hangLeft = null;

        //Servo servo1 = null;
        //Servo wrist = null;

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        BoatPoleAngle = (DcMotorEx) hardwareMap.get(DcMotor.class, "boatAngle");
        hangRight = hardwareMap.get(DcMotor.class, "hangRight");
        hangLeft = hardwareMap.get(DcMotor.class, "hangLeft");
        //servo1 = hardwareMap.get(Servo.class, "servo1");
        //wrist = hardwareMap.get(Servo.class, "wrist");

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        BoatPoleAngle.setDirection(DcMotor.Direction.FORWARD);
        hangRight.setDirection(DcMotor.Direction.FORWARD);
        hangLeft.setDirection(DcMotor.Direction.REVERSE);

        hangRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        //servo1.setPosition(0.5);
        //sleep(2000);
        //servo1.setPosition(0.5);
        rightFront.setPower(0.3);
        rightRear.setPower(0.3);
        leftFront.setPower(0.3);
        leftRear.setPower(0.3);
        //sleep(2250);
        sleep(3000);
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);

        /*BoatPoleAngle.setPower(0.1);
        //sleep(500);
        wrist.setPosition(0.4);
        sleep(500);
        servo1.setPosition(0.3);

        */


        }
    }

