package org.firstinspires.ftc.teamcode.Youtube;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.drive.LinearActuator;

/** Configuration File
 * Control Hub:
 * Port 00: leftFront
 * Port 01: leftRear
 * Port 02: rightFront
 * Port 03: rightRear
 * Expansion Hub:
 * Port 00: hangMotor
 * Port 01: linearActuator
 * Servo 00: claw
 * Digital 1 linearActuatorZero
 */

/** Controller Breakdown
 * Gamepad 1:
 * leftStickX Robot Strafe
 * leftStickY Robot Drive Forwards and Backwards
 * rightStickX Robot Turn
 * rightBumper Bot Speed = 1
 * rightTrigger Bot Speed = .63
 * leftTrigger Bot Speed = .25
 *
 * Gamepad 2:
 * dpad-Up hangMotor Run Forward
 * dpad-Down hangMotor Run Backwards
 * rightTrigger LinearActuator to Wall Height
 * leftTrigger LinearActuator  to HighChamber Height
 * leftBumper HookFunction
 * buttonA Open Claw
 * buttonB Close Claw
 * rightTriggerButton Reset LinearActuator
 */
//@Disabled
@TeleOp(group = "Primary",name = "MK2DriveCode")
public class JackFirstTest extends LinearOpMode {
//Initialization Variables
    final private double motorZeroPower = 0.0;

//Drive Motors
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;

//Drive Variables
    float turn = 0;
    private double drivePower = 0.5;
    private double MLF = 0;
    private double MLR = 0;
    private double MRF = 0;
    private double MRR = 0;
    private double x = 0;
    private double y = 0;
    private double theta = 0;
    private double power = 0;
    private double sin = 0;
    private double cos = 0;
    private double max = 0.0;

//Linear Actuator Motors
    private DcMotor linearActuator;

//Linear Actuator Servos
    private Servo claw;

//Linear Actuator Sensors
    private TouchSensor linearActuatorZero;
    private VoltageSensor voltageSensor;

//Linear Actuator Variables
    private int linearActuatorUpperLimit = 0;    //Not finalized
    private int linearActuatorLowerLimit = 0;    //Not finalized
    private int linearActuatorPosition;
    private int highChamberPosition = 2243; //Not finalized
    private int wallGrabPosition = 294; //Not finalized
    private int specimenHookDifference = 550;//Not Finalized
    private double benchMarkVoltage;
    private double specimenVoltageTrigger = 1.0; //in whole volts
    private double clawOpenPosition = .3;       //Not finalized
    private double clawClosePosition = .5;  //Not finalized
    private boolean ClawOpen = false;
    private boolean isZeroed = true;
//Hang Motors
    private DcMotor hangMotor;

//Hang Variables
    private int hangAngle;
    private int hangSpeed = 20;
    @Override
    public void runOpMode() throws InterruptedException{
        //Initialization Phase/just run once
            initHardware();
        //Loop before the run button is pressed
        while(!isStarted()) {
            motorTelemetry();
        }
        //things you want to happen once before the program loops
        waitForStart();
        //run while the code is active

        while(opModeIsActive()){
            gamepad1Code();
            gamepad2Code();
            safteyCode();
            motorTelemetry();
        }
    }
    public void gamepad1Code(){
        powerAjustment();
        mecanumMath();
    }
    //Code that calls the functions that use the gamepad1
    public void gamepad2Code(){
        hangControl();
        linearActuatorAjustment();
        clawAjustment();
    }
    //Code that calls the functions that use the gamepad2
    public void motorTelemetry(){
        /* Doesn't do anything until we get encoder wires plugged into the drive motors
        telemetry.addData("MotorRightFront", rightFront.getCurrentPosition());
        telemetry.addData("MotorRightRear", rightRear.getCurrentPosition());
        telemetry.addData("MotorLeftFront", leftFront.getCurrentPosition());
        telemetry.addData("MotorLeftRear", leftRear.getCurrentPosition());
         */
        telemetry.addData("linearActuator", linearActuator.getCurrentPosition());
        telemetry.addData("HangMotor", hangMotor.getCurrentPosition());
        telemetry.addData( "is Zeroed = ", isZeroed);
        telemetry.addData("clawOpen = ", ClawOpen);

        if (claw.getPosition()>.25&&claw.getPosition()<.35){
            telemetry.addData("Claw is ", "open");
        }
        else if (claw.getPosition()>.45&&claw.getPosition()<.55){
            telemetry.addData("Claw is ","closed");
        }
        telemetry.update();

    }
    //Code that outputs the robot information in an understandable way to the driver or coach.
    public void initHardware() {
        initDriveMotors();
        initLinearActuator();
        initHang();
    }
    //Code that initializes the different drive systems.
    public void initDriveMotors(){
        initLeftFront();
        initLeftRear();
        initRightFront();
        initRightRear();
    }
    //Code that runs the methods to initialize the drive motors
    private void safteyCode(){
        if (linearActuatorZero.isPressed()){
            linearActuatorReset();
        }
    }
    //Code that makes sure you don't run the Linear actuator past the stop and will eventually have all the saftey systems on the robot
    public void initLeftFront(){
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setPower(motorZeroPower);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //Initializes the leftFront Motor
    public void initLeftRear(){
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setPower(motorZeroPower);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //Initializes the leftRear Motor
    public void initRightFront(){
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setPower(motorZeroPower);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //Initializes the rightFront Motor
    public void initRightRear(){
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setPower(motorZeroPower);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //Initializes the rightRear Motor
    public void initLinearActuator(){
        initLinearActuatorMotor();
        initClawServo();
        initActuatorSensors();
    }
    //Code that runs the methods to initialize the Linear Actuator Systems
    public void initLinearActuatorMotor(){
        linearActuator = hardwareMap.get(DcMotor.class, "linearActuator");
        linearActuator.setDirection(DcMotor.Direction.REVERSE);
        linearActuator.setPower(motorZeroPower);
        linearActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Eventually Add PID control for this motor
        /*
        linearActuatorRunToPosition(100,.1);
        while(linearActuator.isBusy()){
        }
        linearActuatorResetOperation(-.1);
        */
    }
    //Initializes the linearActuator Motor
    public void initClawServo(){
      claw = hardwareMap.get(Servo.class, "claw");
      claw.setDirection(Servo.Direction.FORWARD);
      claw.setPosition(clawClosePosition);
    }
    //Initializes the Claw Servo
    public void initActuatorSensors(){
        initVoltageSensor();
        initTouchSensors();
    }
    //Code that runs the methods to initialize the Linear Actuator Sensors
    public void initVoltageSensor(){
        voltageSensor = hardwareMap.get(VoltageSensor.class,"Control Hub");
    }
    //Initializes the VoltageSensor on the ControlHub
    public void initTouchSensors(){
        linearActuatorZero = hardwareMap.get(TouchSensor.class,"linearActuatorZero");
    }
    //Initializes the TouchSensor on the ControlHub
    public void initHang(){
        initHangMotor();
    }
    //Code that runs the methods to initialize the Hang Systems
    public void initHangMotor(){
        hangMotor = hardwareMap.get(DcMotor.class, "hangMotor");
        hangMotor.setDirection(DcMotor.Direction.FORWARD);
        hangMotor.setPower(motorZeroPower);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //Code that initializes the Hang Motor
    public void mecanumMath(){
        //Converts Driver Inputs into Power and Angle
        x = (gamepad2.left_stick_x * -1);
        y = gamepad2.left_stick_y;
        turn = -gamepad2.right_stick_x;
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
    }
    //Code that does the math for the drive System and controls the joystick input values
    public void powerAjustment(){
        if (gamepad1.right_bumper) {
            drivePower = drivePower = 1;
        }
        if (gamepad1.right_trigger >= .2) {
            drivePower = drivePower = 0.63;
        }
        if (gamepad1.left_trigger >= .2) {
            drivePower = drivePower = 0.25;
        }
    }
    //Code that controls the Driver Control Drivetrain speed
    public void hangControl(){
        if(gamepad2.dpad_up){
            hangAngle += hangSpeed;
            //hangAngle = Math.min(Math.max(hangAngle, -2500), 0);
            hangRunToPosition(hangAngle,1);
        }
        if(gamepad2.dpad_down){
            hangAngle -= hangSpeed;
            //hangAngle = Math.min(Math.max(hangAngle, -2500), 0);
            hangRunToPosition(hangAngle,1);
        }
    }
    //Code that controls the inputs for the hang arm
    private void hangRunToPosition(int ticks, double power) {
        hangMotor.setTargetPosition(ticks);
        hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangMotor.setPower(power);
    }
    //The method that controls the hang motor with encoders
    private void linearActuatorAjustment(){
        if(gamepad2.right_trigger>.1){
            if (claw.getPosition()==clawClosePosition){
                specimenHook();
            }
            else if (claw.getPosition()==clawOpenPosition){
                linearActuatorRunToPosition(wallGrabPosition,1);
            }
            //while(linearActuator.isBusy()){}
            //don't know how this will affect the drive code, may need to either alter the code or add the drive code to here
            //linearActuatorResetOperation(-.6);
        }
        if(gamepad2.left_trigger>.1){
            linearActuatorRunToPosition(highChamberPosition,1);

        }
        if(gamepad2.left_bumper){
            specimenHook();
        }
        if(gamepad2.left_stick_button){
            linearActuatorResetOperation();
        }
    }
    //The method that has the input controls and logic for the linear Actuator
    private void linearActuatorResetOperation() {
        linearActuatorRunToPosition(-10000, .2);
        isZeroed = false;
        while (linearActuator.isBusy()) {
            motorTelemetry();
            if (linearActuatorZero.isPressed()) {
                isZeroed= true;
                linearActuatorReset();
                break;
            }
        }
    }
    //The method that resets the linear actuator zero takes a @parameter that tells the power for the motor to run back at
    private void linearActuatorReset(){
        linearActuator.setPower(motorZeroPower);
        linearActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //The function that only resets the Motor encoder position, doesn't run the motor backwards.
    private void clawAjustment(){
        if (gamepad2.a){
            claw.setPosition(clawClosePosition);
        }
        if (gamepad2.b){
            claw.setPosition(clawOpenPosition);
        }
    }
    //The code that controls the inputs and actions of the claw
    private void specimenHook(){
        linearActuatorRunToPosition(highChamberPosition,1);
        while(linearActuator.isBusy()) {
        }
        double voltageBenchmark = voltageSensor.getVoltage();
        linearActuatorRunToPosition((highChamberPosition-specimenHookDifference),.3);
        while(linearActuator.isBusy()) {
            if ((voltageBenchmark - voltageSensor.getVoltage()) > specimenVoltageTrigger) {
                claw.setPosition(clawOpenPosition);
            }
        }
        claw.setPosition(clawOpenPosition);
    }
    //The method that runs the operation to hook the specimen
    private void linearActuatorRunToPosition(int ticks, double power){
        linearActuator.setTargetPosition(ticks);
        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearActuator.setPower(power);
    }
    //The method that controls the linearActuator motor with encoders
    private void devMode(){
        rightFront.setPower(motorZeroPower);
        rightRear.setPower(motorZeroPower);
        leftFront.setPower(motorZeroPower);
        leftRear.setPower(motorZeroPower);
        hangMotor.setPower(motorZeroPower);
        linearActuator.setPower(motorZeroPower);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        linearActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        //This method should be used in place of the gamepad1 and gamepad2 it shuts off all motor use and allow manual manipulation of the robot.
        //DevMode only uncomment when you want to use



}

