using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System;
using System.Collections.Concurrent;

public class SimulinkTCPReceiver : MonoBehaviour
{
    // Configuration
    public int port = 55000;
    public string ipAddress = "127.0.0.1";
    
    // The Ship Component to control
    public Transform shipTransform;
    public Transform leftFoil;
    public Transform rightFoil;

    // Threading variables
    private TcpListener listener;
    private Thread listenerThread;
    private ConcurrentQueue<double[]> dataQueue = new ConcurrentQueue<double[]>();
    private bool isRunning = true;

    void Start()
    {
        // Start the background thread for TCP listening
        listenerThread = new Thread(new ThreadStart(ListenForData));
        listenerThread.IsBackground = true;
        listenerThread.Start();
    }

    void ListenForData()
    {
        try
        {
            listener = new TcpListener(IPAddress.Parse(ipAddress), port);
            listener.Start();
            Debug.Log("Server Started. Waiting for Simulink...");

            while (isRunning)
            {
                // Wait for connection
                using (TcpClient client = listener.AcceptTcpClient())
                using (NetworkStream stream = client.GetStream())
                {
                    byte[] buffer = new byte[1024]; // Adjust based on packet size
                    int bytesRead;

                    while ((bytesRead = stream.Read(buffer, 0, buffer.Length)) != 0)
                    {
                        // Assuming Simulink sends Doubles (8 bytes each)
                        // Example: 6 DOF + 2 Flaps = 8 values = 64 bytes
                        double[] receivedValues = ConvertBytesToDoubles(buffer, bytesRead);
                        dataQueue.Enqueue(receivedValues);
                    }
                }
            }
        }
        catch (Exception e)
        {
            Debug.Log($"TCP Error: {e.Message}");
        }
    }

    void Update()
    {
        // Apply the latest data to the ship in the Main Unity Thread
        if (dataQueue.TryDequeue(out double[] data))
        {
            if (data.Length >= 6)
            {
                // Apply Position (Swap Y and Z because Unity is Y-up, Simulink is usually Z-up)
                // You might need to scale these values if Simulink uses meters and Unity uses units
                shipTransform.position = new Vector3((float)data[0], (float)data[2], (float)data[1]);
                
                // Apply Rotation
                shipTransform.rotation = Quaternion.Euler((float)data[3], (float)data[5], (float)data[4]);

                // Apply Flaps (if data exists)
                if (data.Length >= 8 && leftFoil != null && rightFoil != null)
                {
                     leftFoil.localRotation = Quaternion.Euler((float)data[6], 0, 0);
                     rightFoil.localRotation = Quaternion.Euler((float)data[7], 0, 0);
                }
            }
        }
    }

    // Helper: Convert raw bytes to double array
    private double[] ConvertBytesToDoubles(byte[] bytes, int totalBytes)
    {
        int doubleSize = sizeof(double);
        int count = totalBytes / doubleSize;
        double[] result = new double[count];

        for (int i = 0; i < count; i++)
        {
            // Note: Simulink is often Big-Endian, C# is Little-Endian. 
            // You may need to Array.Reverse the slice if numbers look garbage.
            result[i] = BitConverter.ToDouble(bytes, i * doubleSize);
        }
        return result;
    }

    void OnApplicationQuit()
    {
        isRunning = false;
        listener?.Stop();
        listenerThread?.Abort();
    }
}