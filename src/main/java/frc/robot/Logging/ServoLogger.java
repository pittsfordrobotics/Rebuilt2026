// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Logging;

import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.Servo;

/** Add your docs here. */
public class ServoLogger extends ClassSpecificLogger <Servo>{
    public ServoLogger(){
        super(Servo.class);
    }

    @Override
    public void update(EpilogueBackend backend, Servo servo){
        //Log motor IDs
        backend.log("servoposition", servo.getPosition());
    }
}

