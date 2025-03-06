package org.firstinspires.ftc.teamcode.Youtube;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


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
 * rightTrigger Wall intake Position
 * leftTrigger Outake Position
 * buttonA Open Claw
 * buttonB Close Claw
 * leftTriggerButton Reset LinearActuator
 */

@TeleOp(name = "MK4DriveCode", group = "Primary")
public class MK4DriveCode extends LinearOpMode {
    private RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize robot hardware
        robot.init(hardwareMap);

        // Reset linear actuator if needed
        if (!robot.isLinearActuatorZeroed()) {
            linearActuatorResetOperation();
        }

        // Initialize linear actuator position
        RobotHardware.initializePosition();

        // Wait for start
        while (!isStarted()) {
            telemetryUpdate();
        }
        waitForStart();

        while (opModeIsActive()) {
            driveControl();
            hangControl();
            linearActuatorControl();
            clawControl();
            telemetryUpdate();
        }
    }

    private void driveControl() {
        // Adjust drive power based on triggers/bumpers
        if (gamepad1.right_bumper) {
            robot.setDrivePower(1.0);
        } else if (gamepad1.right_trigger >= 0.2) {
            robot.setDrivePower(0.63);
        } else if (gamepad1.left_trigger >= 0.2) {
            robot.setDrivePower(0.25);
        }

        // Handle mecanum drive
        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;
        robot.mecanumDrive(x, y, turn);
    }

    private void hangControl() {
        if (gamepad2.dpad_up) {
            robot.hangRunToPosition(RobotHardware.END_GAME_HANG_POSITION, 1.0);
        } else if (gamepad2.dpad_down) {
            robot.hangRunToPosition(0, 1.0);
        }
    }

    private void linearActuatorControl() {
        if (gamepad2.right_trigger > 0.5) {
            handleLinearActuatorSequence();
        }
        if (gamepad2.left_stick_button) {
            linearActuatorResetOperation();
        }
    }

    private void handleLinearActuatorSequence() {
        int currentPosition = robot.getLinearActuatorPosition();
        boolean isClawClosed = robot.isClawClosed();

        if (isClawClosed && isPositionInRange(currentPosition,
                RobotHardware.Positions.WALL_HEIGHT_LINEAR + RobotHardware.WALL_UNHOOK_DIFFERENCE, 20)) {
            // Move to high chamber position
            RobotHardware.highChamber();
        } else if (isClawClosed && isPositionInRange(currentPosition,
                RobotHardware.Positions.HIGH_CHAMBER_LINEAR, 20)) {
            // Open claw and lower
            RobotHardware.specimenUnhook();
        } else if (!isClawClosed && isPositionInRange(currentPosition,
                RobotHardware.Positions.HIGH_CHAMBER_LINEAR - RobotHardware.WALL_UNHOOK_DIFFERENCE, 20)) {
            // Move to wall height position
            RobotHardware.wallHeight();
        } else if (/*!isClawClosed && */isPositionInRange(currentPosition,
                RobotHardware.Positions.WALL_HEIGHT_LINEAR, 20)) {
            // Close claw and unhook specimen
            RobotHardware.wallUnhook();
        }
    }

    private boolean isPositionInRange(int position, int target, int tolerance) {
        return position >= (target - tolerance) && position <= (target + tolerance);
    }

    private void clawControl() {
        if (gamepad2.right_bumper) {
            robot.setClawPosition(RobotHardware.CLAW_CLOSE_POSITION);
        }
        if (gamepad2.left_bumper) {
            robot.setClawPosition(RobotHardware.CLAW_OPEN_POSITION);
        }
    }

    private void linearActuatorResetOperation() {
        robot.linearActuatorRunToPosition(-10000, 0.2);
        while (robot.isLinearActuatorBusy()) {
            if (robot.isLinearActuatorZeroed()) {
                robot.linearActuatorReset();
                break;
            }
            telemetryUpdate();
        }
    }

    private void telemetryUpdate() {
        int[] hangPositions = robot.getHangMotorPositions();
        telemetry.addData("HangRight", hangPositions[0]);
        telemetry.addData("HangLeft", hangPositions[1]);
        telemetry.addData("is Zeroed", robot.getIsZeroed());
        telemetry.addData("linearActuator", robot.getLinearActuatorPosition());
        telemetry.addData("Claw is", robot.isClawClosed() ? "closed" : "open");
        telemetry.update();
    }
}