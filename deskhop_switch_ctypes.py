# deskhop_switch_ctypes.py (diagnostic build)
# Zero-deps Windows HID Feature/Output sender + enumerator using ctypes.

import sys
import argparse
import ctypes
from ctypes import wintypes

# --- Constants ---
DIGCF_PRESENT = 0x00000002
DIGCF_DEVICEINTERFACE = 0x00000010

GENERIC_READ  = 0x80000000
GENERIC_WRITE = 0x40000000
FILE_SHARE_READ  = 0x00000001
FILE_SHARE_WRITE = 0x00000002
OPEN_EXISTING    = 3

HIDP_STATUS_SUCCESS = 0x00110000
INVALID_HANDLE_VALUE = wintypes.HANDLE(-1).value

def _detail_cbsize():
    return 8 if ctypes.sizeof(ctypes.c_void_p) == 8 else 6

# --- Structs ---
class GUID(ctypes.Structure):
    _fields_ = [
        ("Data1", wintypes.DWORD),
        ("Data2", wintypes.WORD),
        ("Data3", wintypes.WORD),
        ("Data4", ctypes.c_ubyte * 8),
    ]

class SP_DEVICE_INTERFACE_DATA(ctypes.Structure):
    _fields_ = [
        ("cbSize", wintypes.DWORD),
        ("InterfaceClassGuid", GUID),
        ("Flags", wintypes.DWORD),
        ("Reserved", ctypes.c_void_p),
    ]

class SP_DEVICE_INTERFACE_DETAIL_DATA_W(ctypes.Structure):
    _fields_ = [
        ("cbSize", wintypes.DWORD),
        ("DevicePath", wintypes.WCHAR * 1000)
    ]

class HIDD_ATTRIBUTES(ctypes.Structure):
    _fields_ = [
        ("Size", wintypes.DWORD),
        ("VendorID", wintypes.USHORT),
        ("ProductID", wintypes.USHORT),
        ("VersionNumber", wintypes.USHORT),
    ]

class HIDP_CAPS(ctypes.Structure):
    _fields_ = [
        ("Usage", wintypes.USHORT),
        ("UsagePage", wintypes.USHORT),
        ("InputReportByteLength", wintypes.USHORT),
        ("OutputReportByteLength", wintypes.USHORT),
        ("FeatureReportByteLength", wintypes.USHORT),
        ("NumberLinkCollectionNodes", wintypes.USHORT),
        ("NumberInputButtonCaps", wintypes.USHORT),
        ("NumberInputValueCaps", wintypes.USHORT),
        ("NumberInputDataIndices", wintypes.USHORT),
        ("NumberOutputButtonCaps", wintypes.USHORT),
        ("NumberOutputValueCaps", wintypes.USHORT),
        ("NumberOutputDataIndices", wintypes.USHORT),
        ("NumberFeatureButtonCaps", wintypes.USHORT),
        ("NumberFeatureValueCaps", wintypes.USHORT),
        ("NumberFeatureDataIndices", wintypes.USHORT),
    ]

# --- DLLs ---
setupapi = ctypes.WinDLL("setupapi", use_last_error=True)
hid = ctypes.WinDLL("hid", use_last_error=True)
kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)

# --- Typedefs ---
HDEVINFO = ctypes.c_void_p
PGUID = ctypes.POINTER(GUID)

# --- Prototypes ---
setupapi.SetupDiGetClassDevsW.argtypes = [PGUID, wintypes.LPCWSTR, wintypes.HWND, wintypes.DWORD]
setupapi.SetupDiGetClassDevsW.restype = HDEVINFO

setupapi.SetupDiEnumDeviceInterfaces.argtypes = [HDEVINFO, ctypes.c_void_p, PGUID, wintypes.DWORD, ctypes.POINTER(SP_DEVICE_INTERFACE_DATA)]
setupapi.SetupDiEnumDeviceInterfaces.restype = wintypes.BOOL

setupapi.SetupDiGetDeviceInterfaceDetailW.argtypes = [HDEVINFO, ctypes.POINTER(SP_DEVICE_INTERFACE_DATA), ctypes.c_void_p, wintypes.DWORD, ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p]
setupapi.SetupDiGetDeviceInterfaceDetailW.restype = wintypes.BOOL

setupapi.SetupDiDestroyDeviceInfoList.argtypes = [HDEVINFO]
setupapi.SetupDiDestroyDeviceInfoList.restype = wintypes.BOOL

hid.HidD_GetHidGuid.argtypes = [PGUID]
hid.HidD_GetHidGuid.restype = None

hid.HidD_GetAttributes.argtypes = [wintypes.HANDLE, ctypes.POINTER(HIDD_ATTRIBUTES)]
hid.HidD_GetAttributes.restype = wintypes.BOOL

hid.HidD_SetFeature.argtypes = [wintypes.HANDLE, ctypes.c_void_p, wintypes.ULONG]
hid.HidD_SetFeature.restype = wintypes.BOOL

hid.HidD_SetOutputReport.argtypes = [wintypes.HANDLE, ctypes.c_void_p, wintypes.ULONG]
hid.HidD_SetOutputReport.restype = wintypes.BOOL

hid.HidD_GetPreparsedData.argtypes = [wintypes.HANDLE, ctypes.POINTER(ctypes.c_void_p)]
hid.HidD_GetPreparsedData.restype = wintypes.BOOL

hid.HidD_FreePreparsedData.argtypes = [ctypes.c_void_p]
hid.HidD_FreePreparsedData.restype = wintypes.BOOL

hid.HidP_GetCaps.argtypes = [ctypes.c_void_p, ctypes.POINTER(HIDP_CAPS)]
hid.HidP_GetCaps.restype = wintypes.ULONG

kernel32.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD, ctypes.c_void_p, wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE]
kernel32.CreateFileW.restype = wintypes.HANDLE
kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
kernel32.CloseHandle.restype = wintypes.BOOL

# --- Helpers ---
def _get_hid_guid():
    g = GUID()
    hid.HidD_GetHidGuid(ctypes.byref(g))
    return g

def _enumerate_hid_paths():
    guid = _get_hid_guid()
    hset = setupapi.SetupDiGetClassDevsW(ctypes.byref(guid), None, None, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE)
    if not hset:
        raise OSError(ctypes.get_last_error(), "SetupDiGetClassDevsW failed")
    try:
        i = 0
        while True:
            data = SP_DEVICE_INTERFACE_DATA()
            data.cbSize = ctypes.sizeof(SP_DEVICE_INTERFACE_DATA)
            if not setupapi.SetupDiEnumDeviceInterfaces(hset, None, ctypes.byref(guid), i, ctypes.byref(data)):
                break
            i += 1

            required = wintypes.DWORD(0)
            setupapi.SetupDiGetDeviceInterfaceDetailW(hset, ctypes.byref(data), None, 0, ctypes.byref(required), None)
            needed = required.value or ctypes.sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W)

            FIXED = ctypes.sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W)
            cbSize = _detail_cbsize()

            if needed <= FIXED:
                detail = SP_DEVICE_INTERFACE_DETAIL_DATA_W()
                detail.cbSize = cbSize
                ok = setupapi.SetupDiGetDeviceInterfaceDetailW(hset, ctypes.byref(data), ctypes.byref(detail), FIXED, ctypes.byref(required), None)
                if ok:
                    yield detail.DevicePath
            else:
                raw = ctypes.create_string_buffer(needed)
                ctypes.cast(raw, ctypes.POINTER(wintypes.DWORD)).contents.value = cbSize
                ok = setupapi.SetupDiGetDeviceInterfaceDetailW(hset, ctypes.byref(data), ctypes.byref(raw), needed, ctypes.byref(required), None)
                if ok:
                    base = ctypes.addressof(raw)
                    path_addr = base + ctypes.sizeof(wintypes.DWORD)
                    yield ctypes.wstring_at(path_addr)
    finally:
        setupapi.SetupDiDestroyDeviceInfoList(hset)

def _open(path):
    h = kernel32.CreateFileW(path, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, None, OPEN_EXISTING, 0, None)
    return h

def _attrs(h):
    a = HIDD_ATTRIBUTES()
    a.Size = ctypes.sizeof(HIDD_ATTRIBUTES)
    if not hid.HidD_GetAttributes(h, ctypes.byref(a)):
        return None
    return a

def _caps(h):
    ppd = ctypes.c_void_p()
    if not hid.HidD_GetPreparsedData(h, ctypes.byref(ppd)):
        return None
    try:
        caps = HIDP_CAPS()
        s = hid.HidP_GetCaps(ppd, ctypes.byref(caps))
        if s != HIDP_STATUS_SUCCESS:
            return None
        return caps
    finally:
        hid.HidD_FreePreparsedData(ppd)

def list_devices(vid, pid):
    found = False
    for path in _enumerate_hid_paths():
        h = _open(path)
        if int(h) == INVALID_HANDLE_VALUE or not h:
            continue
        try:
            a = _attrs(h)
            if not a or a.VendorID != vid or a.ProductID != pid:
                continue
            found = True
            c = _caps(h)
            print("----")
            print(path)
            if c:
                print(f"  Caps: In={c.InputReportByteLength}  Out={c.OutputReportByteLength}  Feat={c.FeatureReportByteLength}")
            else:
                print("  Caps: <unavailable>")
        finally:
            kernel32.CloseHandle(h)
    if not found:
        print(f"No HID interfaces found for VID=0x{vid:04X} PID=0x{pid:04X}")

def send_to_path(path, report_id, payload):
    h = _open(path)
    if int(h) == INVALID_HANDLE_VALUE or not h:
        print(f"Open failed: {path}")
        return False
    try:
        c = _caps(h)
        if not c:
            print("  Caps unavailable")
            return False
        feat_len = int(c.FeatureReportByteLength)
        out_len  = int(c.OutputReportByteLength)
        print(f"  Caps: In={c.InputReportByteLength}  Out={out_len}  Feat={feat_len}")

        sent = False
        if feat_len > 0:
            buf = (ctypes.c_ubyte * feat_len)()
            buf[0] = report_id & 0xFF
            if feat_len > 1:
                buf[1] = payload & 0xFF
            ok = hid.HidD_SetFeature(h, ctypes.byref(buf), feat_len)
            print(f"  HidD_SetFeature -> {bool(ok)}")
            sent = bool(ok)

        if not sent and out_len > 0:
            buf2 = (ctypes.c_ubyte * out_len)()
            buf2[0] = report_id & 0xFF
            if out_len > 1:
                buf2[1] = payload & 0xFF
            ok2 = hid.HidD_SetOutputReport(h, ctypes.byref(buf2), out_len)
            print(f"  HidD_SetOutputReport -> {bool(ok2)}")
            sent = bool(ok2)

        return sent
    finally:
        kernel32.CloseHandle(h)

def send_feature_to_vid_pid(vid, pid, report_id, payload):
    for path in _enumerate_hid_paths():
        h = _open(path)
        if int(h) == INVALID_HANDLE_VALUE or not h:
            continue
        try:
            a = _attrs(h)
            if not a or a.VendorID != vid or a.ProductID != pid:
                continue
        finally:
            kernel32.CloseHandle(h)
        print(f"Trying: {path}")
        if send_to_path(path, report_id, payload):
            print("OK")
            return True
    print("Failed to send on any matching HID path.")
    return False

def parse_args(argv):
    p = argparse.ArgumentParser(description="DeskHop HID diag (ctypes).")
    p.add_argument("--vid", type=lambda s: int(s, 16) if s.lower().startswith("0x") else int(s), default=0x1209)
    p.add_argument("--pid", type=lambda s: int(s, 16) if s.lower().startswith("0x") else int(s), default=0xC000)
    p.add_argument("--report-id", type=lambda s: int(s, 16) if s.lower().startswith("0x") else int(s), default=0x10)
    g = p.add_mutually_exclusive_group()
    g.add_argument("--cmd", choices=["toggle", "a", "b"])
    g.add_argument("--value", type=lambda s: int(s, 16) if s.lower().startswith("0x") else int(s))
    p.add_argument("--list", action="store_true", help="List HID interfaces for VID/PID and show caps")
    p.add_argument("--path", help="Send to a specific HID device path")
    return p.parse_args(argv)

def main():
    if sys.platform != "win32":
        print("Run this on Windows.")
        return 1
    args = parse_args(sys.argv[1:])
    value = None
    if args.value is not None:
        value = args.value & 0xFF
    elif args.cmd:
        value = {"toggle": 0x01, "a": 0x02, "b": 0x03}[args.cmd]

    if args.list:
        list_devices(args.vid, args.pid)
        return 0

    if value is None:
        print("Need --cmd or --value (or use --list).")
        return 2

    if args.path:
        ok = send_to_path(args.path, args.report_id, value)
        return 0 if ok else 3
    else:
        ok = send_feature_to_vid_pid(args.vid, args.pid, args.report_id, value)
        return 0 if ok else 3

if __name__ == "__main__":
    sys.exit(main())
