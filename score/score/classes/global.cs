using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Net.Sockets;

namespace WindowsFormsApp1.classes
{
    public class G//global
    {
        public static TcpClient socket;
        public static NetworkStream clientStream;
        public static string myName;
        ~G()
        {
            try
            {
                socket.Close();
            }
            catch { }
        }
    }
}
