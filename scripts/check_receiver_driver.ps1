[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

function Get-UsbSerialFamily {
    param([string]$VidPid)

    switch -Regex ($VidPid) {
        'VID_0483&PID_5740' { return 'STM32 USB CDC (ZooidReceiver or e-puck2 main MCU)' }
        'VID_1D50&PID_6018' { return 'e-puck2 programmer CDC' }
        'VID_10C4' { return 'Silicon Labs CP210x' }
        'VID_1A86|VID_4348' { return 'WCH CH34x/CH91xx' }
        'VID_0403' { return 'FTDI USB serial' }
        'VID_2341|VID_2A03' { return 'Arduino USB serial' }
        default { return 'Unknown; identify the receiver before installing a driver' }
    }
}

$knownUsbSerialVendors = '^USB\\VID_(0483|1D50|10C4|1A86|4348|0403|2341|2A03)&'
$devices = Get-PnpDevice -PresentOnly | Where-Object {
    $_.Class -eq 'Ports' -or
    $_.InstanceId -match $knownUsbSerialVendors -or
    ($_.InstanceId -match '^USB\\VID_' -and $_.Status -ne 'OK')
}

if (-not $devices) {
    Write-Warning 'No USB serial receiver is present. Plug in the Roundup receiver and run this script again.'
    return
}

$result = foreach ($device in $devices) {
    $instanceId = $device.InstanceId
    $hardwareIds = @((Get-PnpDeviceProperty -InstanceId $instanceId -KeyName DEVPKEY_Device_HardwareIds -ErrorAction SilentlyContinue).Data) -join ';'
    $compatibleIds = @((Get-PnpDeviceProperty -InstanceId $instanceId -KeyName DEVPKEY_Device_CompatibleIds -ErrorAction SilentlyContinue).Data) -join ';'
    $identityText = "$hardwareIds;$compatibleIds;$instanceId"
    $vidPid = [regex]::Match($identityText, 'VID_[0-9A-Fa-f]{4}&PID_[0-9A-Fa-f]{4}(?:&MI_[0-9A-Fa-f]{2})?').Value.ToUpperInvariant()
    $isCdc = $compatibleIds -match 'USB\\Class_02&SubClass_02'
    $service = (Get-PnpDeviceProperty -InstanceId $instanceId -KeyName DEVPKEY_Device_Service -ErrorAction SilentlyContinue).Data
    $driverInf = (Get-PnpDeviceProperty -InstanceId $instanceId -KeyName DEVPKEY_Device_DriverInfPath -ErrorAction SilentlyContinue).Data
    $driverVersion = (Get-PnpDeviceProperty -InstanceId $instanceId -KeyName DEVPKEY_Device_DriverVersion -ErrorAction SilentlyContinue).Data
    $busName = (Get-PnpDeviceProperty -InstanceId $instanceId -KeyName DEVPKEY_Device_BusReportedDeviceDesc -ErrorAction SilentlyContinue).Data
    $comPort = [regex]::Match([string]$device.FriendlyName, 'COM\d+').Value

    [pscustomobject]@{
        Status = $device.Status
        Class = $device.Class
        Name = $device.FriendlyName
        BusName = $busName
        VID_PID_MI = $vidPid
        CDC_ACM = $isCdc
        Service = $service
        DriverINF = $driverInf
        DriverVersion = $driverVersion
        COM = $comPort
        Family = Get-UsbSerialFamily $vidPid
        ReadyForQSerialPort = ($device.Status -eq 'OK' -and $device.Class -eq 'Ports' -and [bool]$comPort)
    }
}

$result | Format-List
