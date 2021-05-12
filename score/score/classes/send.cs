using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WindowsFormsApp1.classes
{
    public static class send
    {
        private static string getStringFromServer()
        {
            byte[] buffer = new byte[5], data = new byte[4096];
            int bytesRead = G.clientStream.Read(buffer, 0, 5);
            int byteRead = G.clientStream.Read(data, 0, int.Parse(System.Text.Encoding.UTF8.GetString(buffer).Substring(1)));
            return System.Text.Encoding.UTF8.GetString(buffer) + System.Text.Encoding.UTF8.GetString(data);//get the string
        }
        public static void start()
        {
            Byte[] sendBytes = Encoding.UTF8.GetBytes("Is anybody there?");
            G.clientStream.Write(sendBytes, 0, sendBytes.Length);
            G.clientStream.Flush();
            // Reads NetworkStream into a byte buffer.
            byte[] bytes = new byte[G.socket.ReceiveBufferSize];

            // Read can return anything from 0 to numBytesToRead.
            // This method blocks until at least one byte is read.
            G.clientStream.Read(bytes, 0, (int)G.socket.ReceiveBufferSize);

            // Returns the data received from the host to the console.
            string returndata = Encoding.UTF8.GetString(bytes);

            Console.WriteLine("This is what the host returned to you: " + returndata);
        }
        public static void logOut()
        {
            byte[] buffer = new ASCIIEncoding().GetBytes("0000");//logout
            G.clientStream.Write(buffer, 0, buffer.Length);//send
            G.clientStream.Flush();
            getStringFromServer();
            G.socket.Close();
        }
    }      
}
