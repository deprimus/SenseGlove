/**
 * SenseGlove Driver
 * (C) 2023 UnexomWid
 */
using System;
using System.Linq;
using System.Drawing;
using System.IO.Ports; // This is deprecated in .NET 5+
using System.Threading;
using System.Management;
using System.Runtime.InteropServices;

namespace Sense
{
    internal class Program
    {
        // This is the PNPDeviceId prefix for RPI Pico
        const string DEVICE_ID_PREFIX = @"USB\VID_2E8A&PID_000A";

        enum Mode
        {
            // Will execute something when it detects UP/DOWN/LEFT/RIGHT gestures
            // (change the functions below to customize this)
            GestureDetection,

            // Will act like a mouse, making the glove control the cursor
            Mouse
        }

        static Mode mode = Mode.Mouse;

        // SenseGlove protocol opcodes
        enum Opcode : byte
        {
            // Request data from the device
            // (replies with sizeof(UInt16) * 6 + 1 bytes)
            Poll = 0x0,

            // Makes the device calibrate its sensor
            Calibrate = 0x1,

            // Synchronizes with the device
            // (replies with 0xFF)
            Sync = 0xFF
        }

        static void OnUp()
        {
            Console.WriteLine("\n-- UP --");
            Console.Beep();
        }
        static void OnDown()
        {
            Console.WriteLine("\n-- DOWN --");
            Console.Beep(1200, 200);
        }
        static void OnLeft()
        {
            Console.WriteLine("\n-- LEFT --");
            Console.Beep(1400, 200);
        }
        static void OnRight()
        {
            Console.WriteLine("\n-- RIGHT --");
            Console.Beep(1600, 200);
        }

        static short ReadInt16(SerialPort port)
        {
            byte[] data = new byte[2];

            while (port.BytesToRead < 2) ;

            port.Read(data, 0, 2);

            return BitConverter.ToInt16(data, 0);
        }

        static byte ReadUInt8(SerialPort port)
        {
            byte[] data = new byte[1];

            while (port.BytesToRead < 1) ;

            port.Read(data, 0, 1);

            return data[0];
        }

        [DllImport("user32.dll")]
        static extern bool GetCursorPos(out Point lpPoint);

        [DllImport("user32.dll")]
        static extern bool SetCursorPos(int x, int y);

        [DllImport("user32.dll", EntryPoint = "mouse_event")]
        public static extern void MouseEvent(uint dwFlags, int dx, int dy, uint dwData, int dwExtraInfo);

        static void Main(string[] args)
        {
            while (true)
            {
                try
                {
                    Console.Clear();
                    Console.WriteLine("[SenseGlove Driver]\n");
                    Console.WriteLine("Scanning Serial Ports...\n");
                    Console.WriteLine(string.Format("Looking for PNPDeviceId={0}*", DEVICE_ID_PREFIX));

                    string portName = null;
                    var ports = SerialPort.GetPortNames();

                    foreach (var p in ports)
                    {
                        using (var searcher = new ManagementObjectSearcher(string.Format("SELECT * FROM Win32_SerialPort WHERE DeviceID = '{0}'", p)))
                        {
                            foreach (var obj in searcher.Get())
                            {
                                if (obj["PNPDeviceId"].ToString().StartsWith(DEVICE_ID_PREFIX))
                                {
                                    portName = p;
                                    Console.WriteLine("Found SenseGlove on " + portName);
                                    break;
                                }

                                Console.WriteLine(string.Format("{0} has PNPDeviceId={1}", p, obj["PNPDeviceId"].ToString()));
                            }
                        }

                        if (portName != null)
                        {
                            break;
                        }
                    }

                    if (portName == null)
                    {
                        Console.WriteLine("\nNo SenseGlove device found, rescanning in 5s");
                        Thread.Sleep(5000);
                        continue;
                    }

                    var port = new SerialPort(portName, 115200, Parity.None, 8, StopBits.One);
                    port.Handshake = Handshake.RequestToSend;
                    port.DtrEnable = true;
                    port.Open();

                    // SYNC
                    port.Write(new byte[1] { (byte) Opcode.Sync }, 0, 1);

                    if (ReadUInt8(port) != (byte) Opcode.Sync)
                    {
                        Console.WriteLine("Error: bad sync value");
                        return;
                    }

                    Console.WriteLine("Sync");

                    // CALIBRATE
                    // uncomment this to make the SenseGlove calibrate its sensor
                    //port.Write(new byte[1] { (byte) Opcode.Calibrate }, 0, 1);

                    float[] xVals = new float[30];
                    float[] zVals = new float[30];
                    int index = 0;

                    bool mouseDown = false;

                    Array.Clear(zVals, 0, zVals.Length);

                    while (true)
                    {
                        // POLL
                        port.Write(new byte[1] { (byte) Opcode.Poll }, 0, 1);

                        var rawAccelX = ReadInt16(port);
                        var rawAccelY = ReadInt16(port);
                        var rawAccelZ = ReadInt16(port);

                        var rawGyroX = ReadInt16(port);
                        var rawGyroY = ReadInt16(port);
                        var rawGyroZ = ReadInt16(port);

                        // BOOTSEL button
                        var button = ReadUInt8(port);

                        var accelX = (decimal)Math.Round(2 * ((float)rawAccelX) / (short.MaxValue + 1), 1);
                        var accelY = (decimal)Math.Round(2 * ((float)rawAccelY) / (short.MaxValue + 1), 1);
                        var accelZ = (decimal)Math.Round(2 * ((float)rawAccelZ) / (short.MaxValue + 1), 1);

                        var gyroX = (decimal)Math.Round(250 * ((float)rawGyroX) / (short.MaxValue + 1), 1);
                        var gyroY = (decimal)Math.Round(250 * ((float)rawGyroY) / (short.MaxValue + 1), 1);
                        var gyroZ = (decimal)Math.Round(250 * ((float)rawGyroZ) / (short.MaxValue + 1), 1);

                        var xVal = (float)accelX;
                        var zVal = (float)accelZ;

                        xVals[index] = xVal;
                        zVals[index] = zVal;
                        index = (index + 1) % xVals.Length;

                        var xAvg = xVals.ToList().Average();
                        var zAvg = zVals.ToList().Average();

                        /*
                         * Raw accelerometer values (rawAccelX, etc): -short.MinValue -> short.MaxValue
                         * Accelerometer values (accelX, etc): -2 -> 2
                         * (xVal is accelX but casted to float; use this for calculating stuff)
                         * 
                         * Raw gyroscope values (rawGyroX, etc): -short.MinValue -> short.MaxValue
                         * Gyroscope values (gyroX, etc): -250 -> 250
                         * 
                         * (decimal is used because Console.Log makes it pretty by trimming decimals)
                         * 
                         * xVals: we keep a buffer of 30 sensor values for the X axis accelerometer,
                         *        and we use their average value to predict gestures
                         *        
                         * Keep in mind that the Z axis (aka under the sensor) will measure 1 when the
                         * sensor is sitting and is not rotated, because that's the gravity acceleration.
                         * 
                         * If the sensor is rotated, other axes will be affected by the gravity.
                         */

                        switch (mode)
                        {
                            case Mode.GestureDetection:
                            {
                                if (zVal > 1.95f && zVal > zAvg && zAvg > 0.8f)
                                {
                                    // Detected UP gesture
                                    OnUp();
                                    Array.Clear(xVals, 0, xVals.Length);
                                    Array.Clear(zVals, 0, zVals.Length);
                                }
                                else if (zVal < -1f && zVal < zAvg && zAvg < 0.8f)
                                {
                                    // Detected DOWN gesture
                                    OnDown();
                                    Array.Clear(xVals, 0, xVals.Length);
                                    Array.Clear(zVals, 0, zVals.Length);
                                }

                                else if (xVal > 1.6f && xVal > xAvg && xAvg > 0.2f)
                                {
                                    // Detected RIGHT gesture
                                    OnRight();
                                    Array.Clear(xVals, 0, xVals.Length);
                                    Array.Clear(zVals, 0, zVals.Length);
                                }
                                else if (xVal < -1.6f && xVal < xAvg && xAvg < 0.2f)
                                {
                                    // Detected LEFT gesture
                                    OnLeft();
                                    Array.Clear(xVals, 0, xVals.Length);
                                    Array.Clear(zVals, 0, zVals.Length);
                                }

                                break;
                            }
                            case Mode.Mouse:
                            {
                                // Sensor value history is not enough, so collect more data.
                                // We only check the history for X because it has the same size as the Z history
                                if (xVals.Length < 2)
                                {
                                    break;
                                }

                                Point cursor;
                                GetCursorPos(out cursor);

                                // Detect changes in acceleration
                                // (this also compensates for the gravity acceleration)
                                float diffX = xVal - xVals[xVals.Length - 1];
                                float diffZ = zVal - zVals[zVals.Length - 1];

                                // Deadzone
                                if (Math.Abs(diffX) < 0.1f)
                                {
                                    diffX = 0f;
                                }
                                if (Math.Abs(diffZ) < 0.1f)
                                {
                                    diffZ = 0f;
                                }

                                float gyroXVal = (float)gyroX;
                                float gyroZVal = (float)gyroZ;

                                // Deadzone
                                if (Math.Abs(gyroXVal) < 4)
                                {
                                    gyroXVal = 0f;
                                }
                                if (Math.Abs(gyroZVal) < 4)
                                {
                                    gyroZVal = 0f;
                                }

                                // Complementary filter
                                float alpha = 0.98f;

                                float fusedX = alpha * (gyroXVal + diffX) + (1 - alpha) * gyroXVal;
                                float fusedZ = alpha * (gyroZVal + diffZ) + (1 - alpha) * gyroZVal;

                                // After the fusion, the X axis affects vertical movement
                                // and the Z axis affects horizontal movement

                                int x = (int)(cursor.X - fusedZ);
                                int y = (int)(cursor.Y - fusedX);

                                SetCursorPos(x, y);

                                // Left click
                                if (button == 1)
                                {
                                    // MOUSEEVENTF_LEFTDOWN
                                    MouseEvent(0x0002, 0, 0, 0, 0);
                                    mouseDown = true;
                                }
                                else
                                {
                                    if (mouseDown)
                                    {
                                        mouseDown = false;

                                        // MOUSEEVENTF_LEFTUP
                                        MouseEvent(0x0004, 0, 0, 0, 0);
                                    }
                                }
                                break;
                            }
                        }

                        Console.Clear();
                        Console.WriteLine("Accel X: " + accelX);
                        Console.WriteLine("Accel Y: " + accelY);
                        Console.WriteLine("Accel Z: " + accelZ);

                        Console.WriteLine("Gyro X: " + gyroX);
                        Console.WriteLine("Gyro Y: " + gyroY);
                        Console.WriteLine("Gyro Z: " + gyroZ);

                        Console.WriteLine("Avg X: " + xAvg);
                        Console.WriteLine("Avg Z: " + zAvg);

                        Thread.Sleep(10);
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine(ex.Message);
                    Thread.Sleep(3000);
                }
            }
        }
    }
}
