using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using Microsoft.Win32.SafeHandles;

namespace CareerFair.Robot.Hardware
{
    public interface IRobotUsbTransport : IDisposable
    {
        bool IsConnected { get; }
        string LastError { get; }
        bool Connect(string port);
        void Disconnect();
        bool SendLine(string line);
        bool TryReadLine(out string line);
    }

    /// <summary>
    /// Explicit Windows USB serial transport. It never discovers-and-opens, arms, pings or moves a robot.
    /// SendLine means accepted into a bounded queue; only firmware responses establish delivery.
    /// No queued command or telemetry survives Disconnect/Connect. Dispose on owner destruction.
    /// </summary>
    public sealed class UsbRobotTransport : IRobotUsbTransport
    {
        private const int QueueLimit = 64;
        private readonly object lifecycle = new object();
        private Session session;
        private string lastError;
        private sealed class PendingLine { public byte[] Bytes; public long Ticks; }
        private sealed class Session
        {
            public SafeFileHandle Handle;
            public Thread Worker;
            public volatile bool Stop, Connected;
            public string Error;
            public readonly ConcurrentQueue<string> Incoming = new ConcurrentQueue<string>();
            public readonly ConcurrentQueue<PendingLine> Outgoing = new ConcurrentQueue<PendingLine>();
            public int IncomingCount, OutgoingCount;
        }

        public bool IsConnected { get { Session current = session; return current != null && current.Connected && !current.Stop; } }
        public string LastError { get { Session current = session; return current != null && current.Error != null ? current.Error : lastError; } }

        public static string[] GetPortNames()
        {
#if UNITY_STANDALONE_WIN || UNITY_EDITOR_WIN
            char[] devices = new char[65536];
            uint length = QueryDosDevice(null, devices, (uint)devices.Length);
            if (length == 0) return Array.Empty<string>();
            List<string> ports = new List<string>();
            foreach (string name in new string(devices, 0, (int)length).Split('\0'))
                if (ValidPort(name)) ports.Add(name);
            ports.Sort((a, b) => int.Parse(a.Substring(3)).CompareTo(int.Parse(b.Substring(3))));
            return ports.ToArray();
#else
            return Array.Empty<string>();
#endif
        }

        public bool Connect(string port)
        {
            lock (lifecycle)
            {
                if (session != null) { lastError = "Disconnect the existing session first."; return false; }
                if (!ValidPort(port)) { lastError = "Use a Windows COM port such as COM3."; return false; }
#if UNITY_STANDALONE_WIN || UNITY_EDITOR_WIN
                SafeFileHandle handle = CreateFile("\\\\.\\" + port, 0xC0000000, 0, IntPtr.Zero, 3, 0x80, IntPtr.Zero);
                if (handle.IsInvalid) { lastError = WinError("Open " + port); handle.Dispose(); return false; }
                try
                {
                    DCB state = new DCB { Length = (uint)Marshal.SizeOf(typeof(DCB)) };
                    if (!GetCommState(handle, ref state)) throw new InvalidOperationException(WinError("Read serial settings"));
                    state.BaudRate = 115200;
                    state.Flags = 1; // Binary, all handshakes OFF, DTR_CONTROL_DISABLE, RTS_CONTROL_DISABLE.
                    state.ByteSize = 8; state.Parity = 0; state.StopBits = 0;
                    if (!SetCommState(handle, ref state)) throw new InvalidOperationException(WinError("Set serial settings"));
                    if (!EscapeCommFunction(handle, 6) || !EscapeCommFunction(handle, 4))
                        throw new InvalidOperationException(WinError("Keep DTR/RTS deasserted"));
                    COMMTIMEOUTS timeouts = new COMMTIMEOUTS { ReadIntervalTimeout = uint.MaxValue, WriteTotalTimeoutConstant = 100 };
                    if (!SetCommTimeouts(handle, ref timeouts)) throw new InvalidOperationException(WinError("Set bounded serial timeouts"));
                    if (!PurgeComm(handle, 0x000C)) throw new InvalidOperationException(WinError("Clear old serial buffers"));
                    Session next = new Session { Handle = handle, Connected = true };
                    next.Worker = new Thread(() => Pump(next)) { IsBackground = true, Name = "CareerFair USB CF1" };
                    lastError = null; session = next; next.Worker.Start(); return true;
                }
                catch (Exception error) { handle.Dispose(); lastError = error.Message; return false; }
#else
                lastError = "USB serial transport is available in the Windows editor/player."; return false;
#endif
            }
        }

        public void Disconnect()
        {
            lock (lifecycle)
            {
                Session old = session;
                if (old == null) return;
                old.Stop = true; old.Connected = false;
                // Closing cancels pending native I/O. Firmware watchdog handles loss of host;
                // callers should explicitly send HOLD before disconnect when possible.
                if (old.Worker != null && old.Worker.IsAlive) old.Worker.Join(150);
                old.Handle.Dispose();
                if (old.Worker != null && old.Worker.IsAlive) old.Worker.Join(150);
                if (old.Error != null) lastError = old.Error;
                session = null;
            }
        }

        public bool SendLine(string line)
        {
            if (!CF1Protocol.IsSafeLine(line)) { lastError = "Outgoing line must be printable ASCII, at most 192 bytes, without CR/LF."; return false; }
            Session current = session;
            if (current == null || !current.Connected || current.Stop) { lastError = "USB port is not connected."; return false; }
            if (Interlocked.Increment(ref current.OutgoingCount) > QueueLimit)
            {
                Interlocked.Decrement(ref current.OutgoingCount);
                Fail(current, "Outgoing serial queue overflow; connection stopped."); return false;
            }
            current.Outgoing.Enqueue(new PendingLine { Bytes = Encoding.ASCII.GetBytes(line + "\n"), Ticks = Stopwatch.GetTimestamp() });
            return true;
        }

        public bool TryReadLine(out string line)
        {
            line = null; Session current = session;
            if (current == null || !current.Incoming.TryDequeue(out line)) return false;
            Interlocked.Decrement(ref current.IncomingCount); return true;
        }

        public void Dispose() { Disconnect(); }

        private static bool ValidPort(string port)
        {
            if (port == null || port.Length < 4 || port.Length > 9 || !port.StartsWith("COM", StringComparison.Ordinal)) return false;
            if (port[3] == '0') return false;
            for (int i = 3; i < port.Length; i++) if (port[i] < '0' || port[i] > '9') return false;
            return true;
        }

        private static void Fail(Session target, string error) { target.Error = error; target.Connected = false; target.Stop = true; }

        private static void Pump(Session target)
        {
#if UNITY_STANDALONE_WIN || UNITY_EDITOR_WIN
            byte[] received = new byte[512];
            SerialLineAccumulator lines = new SerialLineAccumulator();
            try
            {
                while (!target.Stop)
                {
                    if (!ReadFile(target.Handle, received, (uint)received.Length, out uint count, IntPtr.Zero))
                        throw new InvalidOperationException(WinError("USB read"));
                    for (int i = 0; i < count; i++)
                    {
                        if (!lines.Push(received[i], out string line, out string error)) throw new InvalidOperationException(error);
                        if (line == null) continue;
                        if (Interlocked.Increment(ref target.IncomingCount) > QueueLimit)
                        { Interlocked.Decrement(ref target.IncomingCount); throw new InvalidOperationException("Incoming serial queue overflow."); }
                        target.Incoming.Enqueue(line);
                    }
                    if (!target.Stop && target.Outgoing.TryDequeue(out PendingLine pending))
                    {
                        Interlocked.Decrement(ref target.OutgoingCount);
                        double age = (Stopwatch.GetTimestamp() - pending.Ticks) / (double)Stopwatch.Frequency;
                        if (age > 0.2) throw new InvalidOperationException("Queued command expired; USB connection stopped.");
                        if (!WriteFile(target.Handle, pending.Bytes, (uint)pending.Bytes.Length, out uint written, IntPtr.Zero) || written != pending.Bytes.Length)
                            throw new InvalidOperationException(WinError("USB write or partial write"));
                    }
                    Thread.Sleep(5);
                }
            }
            catch (Exception error) { if (!target.Stop) Fail(target, error.Message); }
            finally { target.Connected = false; }
#endif
        }

        private static string WinError(string action) => action + " failed (Windows error " + Marshal.GetLastWin32Error() + ").";

        [StructLayout(LayoutKind.Sequential)] private struct DCB
        {
            public uint Length, BaudRate, Flags;
            public ushort Reserved, XonLimit, XoffLimit;
            public byte ByteSize, Parity, StopBits;
            public sbyte XonChar, XoffChar, ErrorChar, EofChar, EventChar;
            public ushort Reserved1;
        }
        [StructLayout(LayoutKind.Sequential)] private struct COMMTIMEOUTS
        {
            public uint ReadIntervalTimeout, ReadTotalTimeoutMultiplier, ReadTotalTimeoutConstant, WriteTotalTimeoutMultiplier, WriteTotalTimeoutConstant;
        }
        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)] private static extern SafeFileHandle CreateFile(string file, uint access, uint share, IntPtr security, uint creation, uint flags, IntPtr template);
        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)] private static extern uint QueryDosDevice(string device, [Out] char[] target, uint max);
        [DllImport("kernel32.dll", SetLastError = true)] [return: MarshalAs(UnmanagedType.Bool)] private static extern bool GetCommState(SafeFileHandle file, ref DCB state);
        [DllImport("kernel32.dll", SetLastError = true)] [return: MarshalAs(UnmanagedType.Bool)] private static extern bool SetCommState(SafeFileHandle file, ref DCB state);
        [DllImport("kernel32.dll", SetLastError = true)] [return: MarshalAs(UnmanagedType.Bool)] private static extern bool EscapeCommFunction(SafeFileHandle file, uint function);
        [DllImport("kernel32.dll", SetLastError = true)] [return: MarshalAs(UnmanagedType.Bool)] private static extern bool SetCommTimeouts(SafeFileHandle file, ref COMMTIMEOUTS timeouts);
        [DllImport("kernel32.dll", SetLastError = true)] [return: MarshalAs(UnmanagedType.Bool)] private static extern bool PurgeComm(SafeFileHandle file, uint flags);
        [DllImport("kernel32.dll", SetLastError = true)] [return: MarshalAs(UnmanagedType.Bool)] private static extern bool ReadFile(SafeFileHandle file, [Out] byte[] buffer, uint count, out uint read, IntPtr overlapped);
        [DllImport("kernel32.dll", SetLastError = true)] [return: MarshalAs(UnmanagedType.Bool)] private static extern bool WriteFile(SafeFileHandle file, byte[] buffer, uint count, out uint written, IntPtr overlapped);
    }

    /// <summary>Bounded streaming ASCII line framing, tested without touching a serial port.</summary>
    public sealed class SerialLineAccumulator
    {
        private readonly StringBuilder text = new StringBuilder(CF1Protocol.MaximumLineBytes);
        private bool carriageReturn;
        public bool Push(byte value, out string line, out string error)
        {
            line = null; error = null;
            if (value == 10)
            {
                if (text.Length != 0) line = text.ToString();
                text.Clear(); carriageReturn = false; return true;
            }
            if (value == 13 && !carriageReturn) { carriageReturn = true; return true; }
            if (carriageReturn || value < 32 || value > 126 || text.Length >= CF1Protocol.MaximumLineBytes)
            { error = "Invalid or oversized serial line; connection stopped."; return false; }
            text.Append((char)value); return true;
        }
    }
}
