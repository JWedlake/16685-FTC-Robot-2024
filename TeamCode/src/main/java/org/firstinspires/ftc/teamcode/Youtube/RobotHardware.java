package org.firstinspires.ftc.teamcode.Youtube;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Configuration File
 * Control Hub:
 * Port 00: leftFront
 * Port 01: leftRear
 * Port 02: rightFront
 * Port 03: rightRear
 * Expansion Hub:
 * Port 00: hangMotorRight
 * Port 01: hangMotorLeft
 * Port 02: linearActuator
 * Servo 00: outakeClaw
 * Servo 01: outakeRotation
 * Servo 02: outakeArm
 * Servo 03: outakeWrist
 * Digital 1: linearActuatorZero
 */

public class RobotHardware {
    // Drive Motors
    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;

    // Linear Actuator Components
    private static DcMotor linearActuator = null;
    private static TouchSensor linearActuatorZero = null;
    private static VoltageSensor voltageSensor = null;

    // Servos
    private static Servo outakeClaw = null;
    private static Servo outakeRotation = null;
    private static Servo outakeWrist = null;
    private static Servo outakeArm = null;

    // Hang Motors
    private static DcMotor hangMotorRight = null;
    private static DcMotor hangMotorLeft = null;

    // Constants
    public static final double CLAW_OPEN_POSITION = 0.3;
    public static final double CLAW_CLOSE_POSITION = 0.5;
    public static final int WALL_UNHOOK_DIFFERENCE = 200;
    public static final int LINEAR_ACTUATOR_UPPER_LIMIT = 3000;
    public static final int LINEAR_ACTUATOR_LOWER_LIMIT = 0;
    public static final int END_GAME_HANG_POSITION = 1000;

    // Servo Range Constants
    private static final int OUTAKE_ROTATION_RANGE = 300;
    private static final int OUTAKE_ROTATION_UPPER = 150;
    private static final int OUTAKE_ROTATION_LOWER = -150;
    private static final int OUTAKE_WRIST_RANGE = 270;
    private static final int OUTAKE_WRIST_UPPER = 135;
    private static final int OUTAKE_WRIST_LOWER = -135;
    private static final int OUTAKE_ARM_RANGE = 270;
    private static final int OUTAKE_ARM_UPPER = 135;
    private static final int OUTAKE_ARM_LOWER = -135;

    // Position Presets
    public static final class Positions {
        // Wall Height
        public static final int WALL_HEIGHT_ROTATION = 90;  // Claw facing up
        public static final int WALL_HEIGHT_WRIST = -90;
        public static final int WALL_HEIGHT_ARM = -110;
        public static final int WALL_HEIGHT_LINEAR = 0;

        // High Chamber
        public static final int HIGH_CHAMBER_ROTATION = -90;  // Claw facing down
        public static final int HIGH_CHAMBER_WRIST = 0;
        public static final int HIGH_CHAMBER_ARM = 85;
        public static final int HIGH_CHAMBER_LINEAR = 350;

        // High Basket
        public static final int HIGH_BASKET_ROTATION = 90;  // Claw facing up
        public static final int HIGH_BASKET_WRIST = 90;
        public static final int HIGH_BASKET_ARM = 25;
        public static final int HIGH_BASKET_LINEAR = 2500;

        // Initialization
        public static final int INIT_ROTATION = -90;  // Claw facing down
        public static final int INIT_WRIST = 0;
        public static final int INIT_ARM = -125;
        public static final int INIT_LINEAR = 0;
    }

    // State tracking
    private static boolean isZeroed = false;
    private double drivePower = 0.5;
    private HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /**
     * Initialize standard Hardware interfaces
     */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        try {
            initializeSensors();
            initializeDriveMotors();
            initializeLinearActuator();
            initializeHangMotors();
            initializeServos();

            setAllMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            throw new RuntimeException("Error initializing hardware: " + e.getMessage());
        }
    }

    /**
     * Initialize drive motors
     */
    public void initializeDriveMotors() {
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        leftRear = hwMap.get(DcMotor.class, "leftRear");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        rightRear = hwMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        setDriveMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Initialize linear actuator system
     */
    public void initializeLinearActuator() {
        linearActuator = hwMap.get(DcMotor.class, "linearActuator");
        linearActuator.setDirection(DcMotor.Direction.REVERSE);
        linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Initialize hang motors
     */
    public void initializeHangMotors() {
        hangMotorRight = hwMap.get(DcMotor.class, "hangMotorRight");
        hangMotorLeft = hwMap.get(DcMotor.class, "hangMotorLeft");

        hangMotorRight.setDirection(DcMotor.Direction.REVERSE);
        hangMotorLeft.setDirection(DcMotor.Direction.FORWARD);

        setHangMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setHangMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Initialize servos
     */
    public void initializeServos() {
        outakeClaw = hwMap.get(Servo.class, "outakeClaw");
        outakeRotation = hwMap.get(Servo.class, "outakeRotation");
        outakeWrist = hwMap.get(Servo.class, "outakeWrist");
        outakeArm = hwMap.get(Servo.class, "outakeArm");
    }

    /**
     * Initialize sensors
     */
    public void initializeSensors() {
        linearActuatorZero = hwMap.get(TouchSensor.class, "linearActuatorZero");
        voltageSensor = hwMap.get(VoltageSensor.class, "Control Hub");
    }

    /**
     * Drive system control methods
     */
    public void setDrivePower(double power) {
        this.drivePower = power;
    }

    public void mecanumDrive(double x, double y, double turn) {
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPower = (power * cos / max + turn);
        double rightFrontPower = (power * sin / max - turn);
        double leftRearPower = (power * sin / max + turn);
        double rightRearPower = (power * cos / max - turn);

        // Normalize powers if they exceed 1.0
        if (((power + Math.abs(turn)) * drivePower) > 1) {
            double scale = power + Math.abs(turn);
            leftFrontPower /= scale;
            rightFrontPower /= scale;
            leftRearPower /= scale;
            rightRearPower /= scale;
        }

        // Apply drive power scaling and set motor powers
        leftFront.setPower(leftFrontPower * drivePower);
        rightFront.setPower(rightFrontPower * drivePower);
        leftRear.setPower(leftRearPower * drivePower);
        rightRear.setPower(rightRearPower * drivePower);
    }

    /**
     * Linear actuator control methods
     */
    public static void linearActuatorRunToPosition(int ticks, double power) {
        linearActuator.setTargetPosition(ticks);
        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearActuator.setPower(power);
    }

    public static void linearActuatorReset() {
        linearActuator.setPower(0);
        linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        isZeroed = true;
    }

    public static void linearActuatorConfiguration(boolean clawOpen, double rotationDegrees, double wristDegrees,
                                            double armDegrees, int linearActuatorTicks, double power) {
        // Set claw position
        outakeClaw.setPosition(clawOpen ? CLAW_OPEN_POSITION : CLAW_CLOSE_POSITION);

        // Apply limits and calculate servo positions
        armDegrees = clamp(armDegrees, OUTAKE_ARM_LOWER, OUTAKE_ARM_UPPER);
        wristDegrees -= armDegrees;
        rotationDegrees = clamp(rotationDegrees, OUTAKE_ROTATION_LOWER, OUTAKE_ROTATION_UPPER);
        wristDegrees = clamp(wristDegrees, OUTAKE_WRIST_LOWER, OUTAKE_WRIST_UPPER);
        linearActuatorTicks = clamp(linearActuatorTicks, LINEAR_ACTUATOR_LOWER_LIMIT, LINEAR_ACTUATOR_UPPER_LIMIT);

        // Convert degrees to servo positions
        double rotation = (rotationDegrees + OUTAKE_ROTATION_RANGE / 2.0) / OUTAKE_ROTATION_RANGE;
        double wrist = (wristDegrees + OUTAKE_WRIST_RANGE / 2.0) / OUTAKE_WRIST_RANGE;
        double arm = (armDegrees + OUTAKE_ARM_RANGE / 2.0) / OUTAKE_ARM_RANGE;

        // Set servo positions
        outakeRotation.setPosition(rotation);
        outakeWrist.setPosition(wrist);
        outakeArm.setPosition(arm);

        // Move linear actuator
        linearActuatorRunToPosition(linearActuatorTicks, power);
    }
    public static void wallHeight(){
        linearActuatorConfiguration(true, Positions.WALL_HEIGHT_ROTATION, Positions.WALL_HEIGHT_WRIST, Positions.WALL_HEIGHT_ARM, Positions.WALL_HEIGHT_LINEAR, 0.5);
    }
    public static void highChamber(){
        linearActuatorConfiguration(false, Positions.HIGH_CHAMBER_ROTATION, Positions.HIGH_CHAMBER_WRIST, Positions.HIGH_CHAMBER_ARM, Positions.HIGH_CHAMBER_LINEAR, 1);
    }
    public static void highBasket(){
        linearActuatorConfiguration(false, Positions.HIGH_BASKET_ROTATION, Positions.HIGH_BASKET_WRIST, Positions.HIGH_BASKET_ARM, Positions.HIGH_BASKET_LINEAR, 1);
    }
    public static void initializePosition(){
        linearActuatorConfiguration(false, Positions.INIT_ROTATION, Positions.INIT_WRIST, Positions.INIT_ARM, Positions.INIT_LINEAR, 0.5);
    }
    public static void specimenUnhook(){
        linearActuatorConfiguration(true, Positions.HIGH_CHAMBER_ROTATION, Positions.HIGH_CHAMBER_WRIST, Positions.HIGH_CHAMBER_ARM, Positions.HIGH_CHAMBER_LINEAR - WALL_UNHOOK_DIFFERENCE, 0.33);
    }
    public static void wallUnhook(){
        linearActuatorConfiguration(false,Positions.WALL_HEIGHT_ROTATION,Positions.WALL_HEIGHT_WRIST,Positions.WALL_HEIGHT_ARM, Positions.WALL_HEIGHT_LINEAR + WALL_UNHOOK_DIFFERENCE,
                0.33);
    }
    /**
     * Hang system control methods
     */
    public void hangRunToPosition(int ticks, double power) {
        hangMotorRight.setTargetPosition(ticks);
        hangMotorLeft.setTargetPosition(ticks);

        hangMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hangMotorRight.setPower(power);
        hangMotorLeft.setPower(power);
    }

    /**
     * Motor mode control methods
     */
    public void setDriveMotorsMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftRear.setMode(mode);
        rightFront.setMode(mode);
        rightRear.setMode(mode);
    }

    public void setHangMotorsMode(DcMotor.RunMode mode) {
        hangMotorRight.setMode(mode);
        hangMotorLeft.setMode(mode);
    }

    public void setAllMotorsMode(DcMotor.RunMode mode) {
        setDriveMotorsMode(mode);
        linearActuator.setMode(mode);
        setHangMotorsMode(mode);
    }

    public void setAllMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        leftRear.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        rightRear.setZeroPowerBehavior(behavior);
        linearActuator.setZeroPowerBehavior(behavior);
        hangMotorRight.setZeroPowerBehavior(behavior);
        hangMotorLeft.setZeroPowerBehavior(behavior);
    }

    public void stopAllMotors() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        linearActuator.setPower(0);
        hangMotorRight.setPower(0);
        hangMotorLeft.setPower(0);
    }

    /**
     * Status check methods
     */
    public boolean isLinearActuatorBusy() {
        return linearActuator.isBusy();
    }

    public boolean areHangMotorsBusy() {
        return hangMotorRight.isBusy() && hangMotorLeft.isBusy();
    }

    public boolean isLinearActuatorZeroed() {
        return linearActuatorZero.isPressed();
    }

    public boolean getIsZeroed() {
        return isZeroed;
    }

    public void setIsZeroed(boolean zeroed) {
        isZeroed = zeroed;
    }

    /**
     * Position getters
     */
    public int getLinearActuatorPosition() {
        return linearActuator.getCurrentPosition();
    }

    public int[] getHangMotorPositions() {
        return new int[]{hangMotorRight.getCurrentPosition(), hangMotorLeft.getCurrentPosition()};
    }

    /**
     * Utility methods
     */
    private static int clamp(int value, int min, int max) {
        return Math.min(Math.max(value, min), max);
    }
    private static double clamp(double value, int min, int max) {
        return Math.min(Math.max(value, min), max);
    }

    /**
     * Drive to position - Used in autonomous mode
     */
    public void driveToPosition(int ticks, double power) {
        leftFront.setTargetPosition(ticks);
        leftRear.setTargetPosition(ticks);
        rightFront.setTargetPosition(ticks);
        rightRear.setTargetPosition(ticks);

        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }

    /**
     * Check if drive motors are busy
     */
    public boolean areDriveMotorsBusy() {
        return leftFront.isBusy() && rightFront.isBusy() &&
                leftRear.isBusy() && rightRear.isBusy();
    }

    /**
     * Claw control methods
     */
    public void setClawPosition(double position) {
        outakeClaw.setPosition(position);
    }

    public boolean isClawClosed() {
        if(outakeClaw.getPosition()==CLAW_CLOSE_POSITION){
            return true;
        }
        return false;
    }
}