package Solitaire.AI;

import java.io.IOException;
import java.io.*;
import java.util.Scanner;

import com.fazecast.jSerialComm.SerialPort;

public class SerialCommunication {
    static int BaudRate = 115200;
    static int DataBits = 8;
    static int StopBits = SerialPort.ONE_STOP_BIT;
    static int Parity = SerialPort.NO_PARITY;

    SerialPort serial;


    public void setupSerialCommunication() {
        SerialPort[] q;
        q = SerialPort.getCommPorts();
        //This iterates through the ports and gives a description and the name of the port
        for (SerialPort a : q) {
            System.out.println(a.getDescriptivePortName() + " : " + a.getSystemPortName());
        }

        System.out.println("Which port do you want?");
        Scanner s = new Scanner(System.in);
        int portnumber = s.nextInt();
        s.close();

        SerialPort serial = SerialPort.getCommPort(q[portnumber].getSystemPortName());
        serial.setComPortParameters(BaudRate, DataBits, StopBits, Parity); // Set parameters for port
        serial.setComPortTimeouts(SerialPort.TIMEOUT_READ_BLOCKING, 1000, 0); // Set timeout for port


        this.serial = serial;
    }

    public void sendCommandToArduino(String commandType, String command) throws IOException, InterruptedException {

        if (serial.openPort()) { // Open port
            System.out.println("Port is opened:)");
            this.serial = serial;
        } else {
            System.out.println("Port is not opened:(");
            this.serial = null;
        }

        String readMsg = "";
        boolean loop = false;

        //Send command type
        while (!loop) {
            System.out.println("SendMessage");
            sendMessage(commandType);
            System.out.println("ReceiveMessage");
            readMsg = receiveMessage();
            System.out.println("CompareMessage");
            loop = compareMsg(commandType, readMsg);
        }

        loop = false;

        while(!loop){
            readMsg = receiveMessage();
            loop = compareMsg("Ready", readMsg);
        }

        loop = false;

        //Send data
        while (!loop) {
            Thread.sleep(1000);
            sendMessage(command);
            readMsg = receiveMessage();

            loop = compareMsg(command, readMsg);
        }

        if(commandType.equals("readCard")){
            loop = false;
            while(!loop){
                readMsg = receiveMessage();
                loop = compareMsg("ready", readMsg);
            }

            try {
                FileWriter writer = new FileWriter("/Users/jannich/Downloads/OpenCV-Playing-Card-Detector-master/ranksuit.txt");

                writer.write("ready");
                writer.close();

            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        loop = false;
        while(!loop){
            readMsg = receiveMessage();
            loop = compareMsg("DONE", readMsg);
        }

        if (serial.closePort()) { // Open port
            System.out.println("Port is closed:)");
        } else {
            System.out.println("Port is still open:(");
        }

        Thread.sleep(100);
    }

    private void sendMessage(String writeMsg) throws IOException, InterruptedException {
        // Write to Arduino
        byte[] writeBuffer = writeMsg.getBytes();
        serial.getOutputStream().write(writeBuffer);
        serial.getOutputStream().flush();
        System.out.println("Message sent: " + writeMsg);
        Thread.sleep(100);
    }

    private String receiveMessage() throws IOException, InterruptedException {

        // Read from Arduino
        byte[] readBuffer = new byte[10];
        int numRead = serial.readBytes(readBuffer, readBuffer.length);

        while(numRead < 1) {
            Thread.sleep(100);
            readBuffer = new byte[10];
            numRead = serial.readBytes(readBuffer, readBuffer.length);
        }

            System.out.print("Read " + numRead + " bytes - ");
            String readMsg = new String(readBuffer);
            System.out.println("Received -> " + readMsg);
            Thread.sleep(100);

        return readMsg;
    }

    private Boolean compareMsg(String writeMsg, String readMsg) throws IOException, InterruptedException {

        Boolean match = true;

        char[] arrReadMsg = readMsg.toCharArray();
        char[] arrWriteMsg = writeMsg.toCharArray();

        for (int i = 0; i < writeMsg.length() - 1; i++) {
            if (arrReadMsg[i] != arrWriteMsg[i]) {
                match = false;
                break;
            }
        }

        if(match) {
            System.out.println("Message match! - Sending confirmation");
            sendMessage("CONFIRMED");
            System.out.println("Confirmation sent");
            return true;
        } else {
            System.out.println("Message no match! - Trying again");
            return false;
        }
    }
}
