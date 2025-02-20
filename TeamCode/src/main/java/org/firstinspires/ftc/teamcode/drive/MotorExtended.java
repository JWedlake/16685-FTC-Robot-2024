package org.firstinspires.ftc.teamcode.drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class MotorExtended extends LinearActuator {
    DcMotor thisMotor = null;
    private boolean encoder = false;
    private boolean controlHub = true;
    private int portNumber = 1;
    private String portName = "Undefined";
    private boolean directionForward = true;
    private String output = null;

    public MotorExtended(boolean encoder, boolean controlHub, int portNumber, String portName, boolean directionForward) {
        DcMotor Motor = null;
        this.thisMotor = Motor;
        this.encoder = encoder;
        this.controlHub = controlHub;
        this.portNumber = portNumber;
        this.portName = portName;
        this.directionForward = directionForward;
    }
    public MotorExtended(){
        DcMotor Motor = null;
        this.thisMotor = Motor;
    }
    public void setEncoder(boolean encoder){
        this.encoder = encoder;
    }
    public void setControlHub(boolean controlHub){
        this.controlHub = controlHub;
    }
    public void setPortNumber(int portNumber){
        this.portNumber = portNumber;
    }
    public void setPortName(String portName){
        this.portName = portName;
    }
    public void setDirectionForward(boolean directionForward){
        this.directionForward = directionForward;
    }


    public void initialize() {
        thisMotor = hardwareMap.get(DcMotor.class, portName);
        if (encoder = true) {
            thisMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            thisMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (directionForward = true) {
            thisMotor.setDirection(DcMotor.Direction.FORWARD);
        } else if (directionForward = false) {
            thisMotor.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    public String toString() {
        output = "Encoder = " + encoder + ", Direction Forwards = " + directionForward + ", Port = ";
        if (controlHub = true) {
            output += "Control hub ";
        } else {
            output += "Expansion hub ";
        }
        output += portNumber;
        return output;
    }
    public String getAddress() {
        output = "";
        if (controlHub = true) {
            output += "Control hub ";
        } else {
            output += "Expansion hub ";
        }
        output += portNumber;
        return output;
    }

}
