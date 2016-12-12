package org.firstinspires.ftc.team11248;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

/**
 * Created by Tony_Air on 12/6/16.
 */

public class MRColorSensorV3 {

    private I2cDeviceSynch colorSynch;
    private byte[] colorCache;

    private int colorNumber;


    public MRColorSensorV3(I2cDevice color_sensor, byte COLOR_SENSOR_ADDR){

        this.colorSynch = new I2cDeviceSynchImpl(color_sensor, I2cAddr.create8bit(COLOR_SENSOR_ADDR), false);
        this.colorSynch.engage();

    }

    public void update(){
        colorCache = colorSynch.read(0x04, 1);
        colorNumber = (colorCache[0] & 0XFF);
    }

    public void writeData(int ireg, int bVal){
        colorSynch.write8(ireg, bVal);

          /*
            Address	Function
            0x03	Command
            0x04	Color Number
            0x05	Red Value
            0x06	Green Value
            0x07	Blue Value
            0x08	White Value
         */
    }

    public void setActiveMode(){ //Senses color
        colorSynch.write8(3,0);
    }

    public void setPassiveMode(){ //Senses light
        colorSynch.write8(3,1);
    }

    public int getColorNumber(){
        this.update();
        return colorNumber;
    }
}
