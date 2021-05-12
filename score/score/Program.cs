using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Net;
using System.Text;
using System.Net.Sockets;
using System.Linq.Expressions;
using System.Threading;
using WindowsFormsApp1.classes;

namespace Client
{
    static class Program
    {
        [STAThread]
        static void Main()
        {
            G.socket = new TcpClient();
            if (!connect(G.socket))//if this failed
                return;
            G.clientStream = G.socket.GetStream();
            try
            {
                Application.EnableVisualStyles();
                Application.SetCompatibleTextRenderingDefault(false);
                Application.Run(new WindowsFormsApp1.Form1());
                send.start();
            }
            catch
            {
                List<Form> openForms = new List<Form>();

                foreach (Form f in Application.OpenForms)
                    openForms.Add(f);

                foreach (Form f in openForms)//close all the windows exepte main
                {
                    if (f.Name != "Menu")
                        f.Close();
                }
            }//catch an error = close the program
            finally
            {
                send.logOut();
            }
        }
        static bool connect(TcpClient client)
        {
            try
            {
                G.socket.Connect(new IPEndPoint(IPAddress.Parse("192.168.137.231"), 65432));//create the socket
                return true;
            }
            catch
            {
                MessageBox.Show("Server is down!");
            }
            return false;
        }
    }
}