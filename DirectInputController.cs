using DSRemapper.Core;
using DSRemapper;
using DSRemapper.DSRMath;
using DSRemapper.Types;
using SharpDX.DirectInput;
using System.Runtime.CompilerServices;

namespace DSRemapper.DirectInput
{
    /// <summary>
    /// Direct input device scanner class
    /// </summary>
    public class DIScanner : IDSRDeviceScanner
    {
        /// <summary>
        /// DirecInput object used in the plugin
        /// </summary>
        internal static readonly SharpDX.DirectInput.DirectInput DI = new();
        /// <inheritdoc/>
        public IDSRInputDeviceInfo[] ScanDevices()
        {
            return DI.GetDevices(DeviceClass.GameControl, DeviceEnumerationFlags.AttachedOnly)
                .Select((x) => { return new DIDeviceInfo(x); }).ToArray();
        }
    }
    /// <summary>
    /// Direct input information class
    /// </summary>
    /// <param name="deviceInstance">Device instance representing the physical controller</param>
    public class DIDeviceInfo(DeviceInstance deviceInstance) : IDSRInputDeviceInfo
    {
        /// <summary>
        /// DirectX DirectInput Device instance class used to create the controller
        /// </summary>
        public DeviceInstance DeviceInstance { get; private set; } = deviceInstance;
        /// <summary>
        /// Gets product GUID of the device
        /// </summary>
        public Guid ProductGuid { get { return DeviceInstance.ProductGuid; } }
        /// <summary>
        /// Gets instance GUID of the device
        /// </summary>
        public Guid InstanceGuid { get { return DeviceInstance.InstanceGuid; } }

        private byte[] ProductBytes { get { return ProductGuid.ToByteArray(); } }
        /// <inheritdoc/>
        public string Id => InstanceGuid.ToString();
        /// <inheritdoc/>
        public string Name { get { return DeviceInstance.ProductName; } }
        /// <summary>
        /// Gets device product id
        /// </summary>
        public int ProductId { get { return BitConverter.ToUInt16(ProductBytes, 2); } }
        /// <summary>
        /// Gets device vendor id
        /// </summary>
        public int VendorId { get { return BitConverter.ToUInt16(ProductBytes, 0); } }

        /// <inheritdoc/>
        public IDSRInputController CreateController() => new DirectInputController(this);
    }
    /// <summary>
    /// Direct Input Controller class
    /// </summary>
    /// <param name="deviceInfo">DirectInput device information requiered to connect the controller</param>
    public class DirectInputController(DIDeviceInfo deviceInfo) : IDSRInputController
    {
        static SharpDX.DirectInput.DirectInput DI => DIScanner.DI;

        private readonly DIDeviceInfo deviceInfo = deviceInfo;
        private readonly Joystick joy = new(DI, deviceInfo.InstanceGuid);
        private readonly IDSRInputReport report = new DefaultDSRInputReport(6, 8, 32, 4, 0, 6, 0);

        /// <inheritdoc/>
        public string Id => deviceInfo.Id;
        /// <inheritdoc/>
        public string Name => deviceInfo.Name;
        /// <inheritdoc/>
        public string Type => "DI";
        /// <inheritdoc/>
        public string ImgPath => "DirectInput.png";
        /// <inheritdoc/>
        public bool IsConnected { get; private set; }
        /// <inheritdoc/>
        public void Connect()
        {
            joy.Acquire();
            IsConnected = true;
        }
        /// <inheritdoc/>
        public void Disconnect()
        {
            IsConnected = false;
            joy.Unacquire();
        }
        /// <inheritdoc/>
        public void Dispose()
        {
            Disconnect();
            GC.SuppressFinalize(this);
        }
        /// <inheritdoc/>
        public IDSRInputReport GetInputReport()
        {
            try
            {
                if (!joy.IsDisposed && IsConnected)
                {
                    joy.Poll();
                    JoystickState state = joy.GetCurrentState();

                    for (int i = 0; i < report.Buttons.Length; i++)
                    {
                        report.Buttons[i] = state.Buttons[i];
                    }

                    report.Axes[0] = AxisToFloat(state.X - short.MaxValue);
                    report.Axes[1] = AxisToFloat(state.Y - short.MaxValue);
                    report.Axes[2] = AxisToFloat(state.Z - short.MaxValue);
                    report.Axes[3] = AxisToFloat(state.RotationZ - short.MaxValue);
                    report.Axes[4] = AxisToFloat(state.RotationX - short.MaxValue);
                    report.Axes[5] = AxisToFloat(state.RotationY - short.MaxValue);

                    report.Sliders[0] = AxisToFloat(state.Sliders[0] - short.MaxValue);
                    report.Sliders[1] = AxisToFloat(state.Sliders[1] - short.MaxValue);
                    report.Sliders[2] = AxisToFloat(state.VelocitySliders[0] - short.MaxValue);
                    report.Sliders[3] = AxisToFloat(state.VelocitySliders[1] - short.MaxValue);
                    report.Sliders[4] = AxisToFloat(state.AccelerationSliders[0] - short.MaxValue);
                    report.Sliders[5] = AxisToFloat(state.AccelerationSliders[1] - short.MaxValue);
                    report.Sliders[6] = AxisToFloat(state.ForceSliders[0] - short.MaxValue);
                    report.Sliders[7] = AxisToFloat(state.ForceSliders[1] - short.MaxValue);

                    report.SixAxes[0] = IntArrToVec(state.VelocityX, state.VelocityY, state.VelocityZ);
                    report.SixAxes[1] = IntArrToVec(state.AngularVelocityX, state.AngularVelocityY, state.AngularVelocityZ);
                    report.SixAxes[2] = IntArrToVec(state.AccelerationX, state.AccelerationY, state.AccelerationZ);
                    report.SixAxes[3] = IntArrToVec(state.AngularAccelerationX, state.AngularAccelerationY, state.AngularAccelerationZ);
                    report.SixAxes[4] = IntArrToVec(state.ForceX, state.ForceY, state.ForceZ);
                    report.SixAxes[5] = IntArrToVec(state.TorqueX, state.TorqueY, state.TorqueZ);

                    for (int i = 0; i < state.PointOfViewControllers.Length; i++)
                    {
                        report.Povs[i].Angle = state.PointOfViewControllers[i] != -1 ? state.PointOfViewControllers[i] / 100 : -1f;
                        report.Povs[i].CalculateButtons();
                    }
                }
            }
            catch { }

            return report;
        }
        /// <inheritdoc/>
        public void SendOutputReport(DefaultDSROutputReport report)
        {

        }

        private static float AxisToFloat(int axis) => (float)axis / (short.MaxValue + (axis > 0 ? 1 : 0));
        private static DSRVector3 IntArrToVec(int x, int y, int z) => new(
            AxisToFloat(x - short.MaxValue), AxisToFloat(y - short.MaxValue), AxisToFloat(z - short.MaxValue));
        private static DSRQuaternion IntArrToQuat(int x, int y, int z) => new(
            AxisToFloat(x - short.MaxValue), AxisToFloat(y - short.MaxValue), AxisToFloat(z - short.MaxValue), 0);
    }
}