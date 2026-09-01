# Windows software setup

This repository is a native Windows Qt/qmake application. It is not a ROS or
ROS 2 package, so ROS, colcon, CMake, Python, and Webots are not required for
building `ZooidManager`.

## Toolchain

The build script defaults to the following legacy kit locations:

- Qt 5.9.7, MinGW 5.3 32-bit: `D:\Qt\5.9.7\mingw53_32`
- MinGW 5.3 32-bit tools: `D:\Qt\Tools\mingw530_32`
- Qt modules: Core, Gui, Widgets, SerialPort, OpenGL, Sql, Multimedia, and
  MultimediaWidgets

Override `-QtRoot` and `-MinGwRoot` if the kit is installed elsewhere. The
build script changes `PATH` only for its own process and does not modify the
machine-wide setup.

## Build and deploy

Keep the repository in an ASCII-only path such as `D:\ros2\Roundup-epuck`.
The Qt 5.9 MinGW resource compiler cannot read `zooid.ico` from a path containing
Chinese characters.

From a PowerShell prompt in the repository root, run:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\build_windows.ps1 -Configuration Release
```

The runnable directory is:

```text
.build-windows-deploy\release
```

The script runs the core controller tests, builds with qmake, runs
`windeployqt`, and copies `json/chargePosition.json`.

On first launch, the application creates `sql/ZooidManager.db3` and a safe
default configuration row beside the executable. The default administrator
password is `admin`; change it in the application before regular use.
Keep the deployment directory writable so the SQLite settings can persist; do
not place it directly under a protected directory such as `Program Files`.

## Receiver and Windows driver

The original ZooidReceiver is an STM32 USB CDC device with VID:PID `0483:5740`
and product name `ZooidReceiver`. Windows 10/11 normally binds a compatible
CDC device to the inbox `usbser.sys` driver. Do not install a legacy Windows
7/8 VCP package unless the actual receiver is plugged in, positively
identified, and still has no working COM port.

The e-puck2 main MCU can use the same VID:PID but reports product name
`e-puck2 STM32F407`; its programmer exposes `1D50:6018`. A working driver does
not make those ports compatible with the custom `Are you?` / `You are?`
ZooidReceiver protocol.

Plug in only the Roundup receiver, then run:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\check_receiver_driver.ps1
```

For the expected receiver, the healthy result is product/bus name
`ZooidReceiver`, `VID_0483&PID_5740`, class `Ports`, service `usbser`, a COM
number, and `ReadyForQSerialPort=True`. The script deliberately omits device
serial numbers.

Do not use Zadig to bind WinUSB/libusbK: Qt SerialPort requires a COM/CDC
driver. Do not install CH340, CP210x, or FTDI packages without a matching
VID/PID. Before a physical run, disconnect unrelated serial devices because the
current application probes nearly every COM port and does not filter by VID/PID.

## References

- [Qt 5.9.7 archive](https://download.qt.io/archive/qt/5.9/5.9.7/)
- [GCtronic e-puck2 USB driver and port documentation](https://www.gctronic.com/doc/index.php/e-puck2#Installing_the_USB_drivers)
- [Microsoft USB CDC / usbser.sys matching](https://learn.microsoft.com/en-us/windows-hardware/drivers/usbcon/usb-driver-installation-based-on-compatible-ids)
- [Upstream ZooidReceiver USB descriptor](https://github.com/ShapeLab/SwarmUI/blob/master/Software/Microcontroller/ZooidReceiver_v2/Src/usbd_desc.c)
