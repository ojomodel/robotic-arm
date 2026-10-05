using System;
using System.Runtime.InteropServices;
using UnityEngine;

namespace CareerFair.Robot
{
    public struct XboxInputFrame
    {
        public bool connected;
        public int controllerIndex;
        public Vector2 leftStick, rightStick;
        public float leftTrigger, rightTrigger;
        public ushort buttons;
        public bool Down(ushort mask) => (buttons & mask) != 0;
        public bool Neutral => leftStick == Vector2.zero && rightStick == Vector2.zero &&
            leftTrigger == 0f && rightTrigger == 0f && buttons == 0;
    }

    /// <summary>Windows' built-in XInput. Reads controllers only; no device output or robot connection.</summary>
    public static class WindowsXboxInput
    {
        public const ushort B = 0x2000, LB = 0x0100, RB = 0x0200;
        [StructLayout(LayoutKind.Sequential)]
        private struct Gamepad
        {
            public ushort buttons;
            public byte leftTrigger, rightTrigger;
            public short lx, ly, rx, ry;
        }
        [StructLayout(LayoutKind.Sequential)]
        private struct State { public uint packet; public Gamepad gamepad; }
#if UNITY_STANDALONE_WIN || UNITY_EDITOR_WIN
        [DllImport("xinput1_4.dll", CallingConvention = CallingConvention.Winapi)]
        private static extern uint XInputGetState(uint index, out State state);
#endif
        public static string Availability { get; private set; } = "Looking for an Xbox controller";

        public static XboxInputFrame Read(int preferredIndex = -1)
        {
#if UNITY_STANDALONE_WIN || UNITY_EDITOR_WIN
            try
            {
                // Keep ownership with one pad; never silently switch pads during a movement.
                int start = preferredIndex >= 0 ? preferredIndex : 0;
                int end = preferredIndex >= 0 ? preferredIndex + 1 : 4;
                for (int i = start; i < end; i++)
                {
                    if (XInputGetState((uint)i, out State state) != 0) continue;
                    Gamepad p = state.gamepad;
                    Availability = "Xbox controller " + (i + 1) + " connected";
                    return new XboxInputFrame { connected = true, controllerIndex = i, buttons = p.buttons,
                        leftStick = Stick(p.lx, p.ly), rightStick = Stick(p.rx, p.ry),
                        leftTrigger = Trigger(p.leftTrigger), rightTrigger = Trigger(p.rightTrigger) };
                }
                Availability = "No Xbox controller detected";
            }
            catch (DllNotFoundException) { Availability = "Windows XInput is unavailable"; }
            catch (EntryPointNotFoundException) { Availability = "Windows XInput is unavailable"; }
#else
            Availability = "Xbox input is supported by this simulator on Windows";
#endif
            return new XboxInputFrame { controllerIndex = -1 };
        }

        public static Vector2 Stick(short x, short y)
        {
            Vector2 v = new Vector2(x < 0 ? x / 32768f : x / 32767f, y < 0 ? y / 32768f : y / 32767f);
            const float deadzone = 0.20f;
            float length = v.magnitude;
            return length <= deadzone ? Vector2.zero : v.normalized * Mathf.Clamp01((length - deadzone) / (1f - deadzone));
        }
        public static float Trigger(byte value) => Mathf.Clamp01((value - 30f) / 225f);
    }
}
