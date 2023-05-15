/*
 * windows backend hotplug header for libusb 1.0
 * Copyright © 2023 Pawel Gorgon <pgorgon@hem-e.com>
 * Parts of this code adapted from libusbx-hp by Pete Batard
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libusbi.h"
#include "windows_common.h"

#include <windows.h>
#include <dbt.h>
#include <string.h>

static HANDLE hotplug_thread = NULL;
static HANDLE hotplug_response = NULL;
static BOOL hotplug_ready = FALSE;
static HWND hHotplugMessage = NULL;


/* User32 dependencies */
DLL_DECLARE_HANDLE(User32);
DLL_DECLARE_FUNC_PREFIXED(WINAPI, ATOM, p, RegisterClassExA, (const WNDCLASSEXA*));
DLL_DECLARE_FUNC_PREFIXED(WINAPI, HDEVNOTIFY, p, RegisterDeviceNotificationA, (HANDLE, LPVOID, DWORD));
DLL_DECLARE_FUNC_PREFIXED(WINAPI, BOOL, p, UnregisterDeviceNotification, (HDEVNOTIFY));
DLL_DECLARE_FUNC_PREFIXED(WINAPI, BOOL, p, UnregisterClassA, (LPCSTR, HINSTANCE));

/*
 * User32 DLL functions
 */
static bool init_dlls(struct libusb_context* ctx)
{
    DLL_GET_HANDLE(ctx, User32);
    DLL_LOAD_FUNC_PREFIXED(User32, p, RegisterClassExA, true);
    DLL_LOAD_FUNC_PREFIXED(User32, p, RegisterDeviceNotificationA, true);
    DLL_LOAD_FUNC_PREFIXED(User32, p, UnregisterDeviceNotification, true);
    DLL_LOAD_FUNC_PREFIXED(User32, p, UnregisterClassA, true);

    return true;
}

static void exit_dlls(void)
{
    DLL_FREE_HANDLE(User32);
}


#define GUID_FORMAT "%08lX-%04hX-%04hX-%02hhX%02hhX-%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX"
#define GUID_ARG(guid) (guid).Data1, (guid).Data2, (guid).Data3, (guid).Data4[0], (guid).Data4[1], (guid).Data4[2], (guid).Data4[3], (guid).Data4[4], (guid).Data4[5], (guid).Data4[6], (guid).Data4[7]

static char *parse_device_interface_path(const char *interface_path)
{
    char *device_id, *guid_start;
    unsigned int i;
    size_t len = 0;

    if (interface_path != NULL)
        len = strlen(interface_path);

    if (len < 4)
    {
        return NULL;
    }

    // Microsoft indiscriminatly uses '\\?\', '\\.\', '##?#" or "##.#" for root prefixes.
    if (((interface_path[0] == '\\') && (interface_path[1] == '\\') && (interface_path[3] == '\\')) ||
        ((interface_path[0] == '#') && (interface_path[1] == '#') && (interface_path[3] == '#')))
    {
        interface_path += 4;
        len -= 4;
    }

    guid_start = strchr(interface_path, '{');
    if (guid_start != NULL)
    {
        len = (guid_start - interface_path) - 1; // One more for separator
    }

    if (len <= 0)
    {
        usbi_err(NULL, "program assertion failed: invalid device interface path");
        return NULL;
    }

    device_id = calloc(len + 1, 1); // One additional for NULL term.
    if (device_id == NULL)
    {
        return NULL;
    }
    strncat(device_id, interface_path, len);

    // Now convert to uppercase and replace '#' with '\'
    for (i = 0; i < len; i++)
    {
        device_id[i] = (char)toupper((int)device_id[i]);
        if (device_id[i] == '#')
            device_id[i] = '\\';
    }

    return device_id;
}

LRESULT CALLBACK message_callback_handle_device_change(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    (void)hWnd;
    (void)message;
    DEV_BROADCAST_DEVICEINTERFACE_A *dev_bdi;
    char* device_id;
    bool connected;

    if (message == DBT_DEVNODES_CHANGED) {
        usbi_dbg(NULL, "EVENT RECEIVED: DBT_DEVNODES_CHANGED");
    }

    dev_bdi = (DEV_BROADCAST_DEVICEINTERFACE_A *)lParam;
    if (dev_bdi->dbcc_devicetype != DBT_DEVTYP_DEVICEINTERFACE)
    {
        return TRUE;
    }

    if ((wParam != DBT_DEVICEARRIVAL) && (wParam != DBT_DEVICEREMOVECOMPLETE))
    {
        usbi_dbg(NULL, "ignoring WM_DEVICECHANGE event %d", wParam);
        return TRUE;
    }

    device_id = parse_device_interface_path(dev_bdi->dbcc_name);
    if (device_id == NULL)
    {
        usbi_dbg(NULL, "could not parse device interface path '%s'", dev_bdi->dbcc_name);
        return TRUE;
    }

    connected = (wParam == DBT_DEVICEARRIVAL);

    usbi_dbg(NULL, "PRO: %s (%s)", device_id, connected ? "CONNECTED" : "DISCONNECTED");

    usbi_dbg(NULL, "PRO: device interface path '%s'", dev_bdi->dbcc_name);
    usbi_dbg(NULL, "PRO: device class GUID: " GUID_FORMAT, GUID_ARG(dev_bdi->dbcc_classguid));

    free(device_id);

    /*
    * The idea is that:
    *   - here invoke device_connected(id) and device_disconnected(id) callbacks on windows backend of each context
    *   - during ctx init force detecting of all devices currently exposed in system and don't dereference them (so they will be available on device connected internal list) - here remember to check removing device after disconnecting. I suppose it is enough to remove code of unref_list in winusb backend. In usbdk backend there is probably bug and disconnected devices are never unreferenced....
    *           Probably it is enough to remove code of managing unref_list and _discdevs (discovered_devs_append) in winusb_get_device_list. In usbdk backend there is probably bug and disconnected devices are never unreferenced....
    *   - in winusb backend:
    *       - manage list of possible device classes and enumerators (that workaround for some HUBs) inside winusb backend context priv
    *       - when device is connected to the system we receive its id and we can just treat it in the same way as they are treated in winusb_get_device_list - to be considered. But probably it is possible to do. 
    *                                       We receive that event for every device and every interface, so we can treat it exactly in the way of get_device_list (receive device, then receive interface and so on)
    *       - when device is removed we can just find it's 'dev' instance looking for session_id - or maybe easier looking for insystem id
    *   - in usbdk backend - check if we can react somehow for that - if not just get new list, disconnect devices which are no longer on list, newly connected will work without any need to change
    */
    return TRUE;
}

/*
 * Hotplug messaging callback
 */
// TODO: Windows limitations mean we cannot detect driverless devices with this
//		 => add another more generic callback for driverless, and check against
//			this one?
LRESULT CALLBACK messaging_callback(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    LRESULT ret = TRUE;

    switch (message)
    {
    case WM_DEVICECHANGE:
        ret = message_callback_handle_device_change(hWnd, message, wParam, lParam);
        break;
    default:
        ret = DefWindowProc(hWnd, message, wParam, lParam);
        break;
    }
    return ret;
}

unsigned __stdcall windows_hotplug_threaded(void *param)
{
    (void)param;
    WNDCLASSEXA wc = {0};
    MSG msg;
    DEV_BROADCAST_DEVICEINTERFACE_A dev_bdi = {0};
    HDEVNOTIFY devnotify = NULL;
    BOOL msg_ret;
    int ret = 1;

#define LIBUSB_MSG_WINDOW_CLASS "libusb_messaging_class"

    wc.cbSize = sizeof(WNDCLASSEX);
    wc.lpfnWndProc = messaging_callback;
    wc.lpszClassName = LIBUSB_MSG_WINDOW_CLASS;

    if (!pRegisterClassExA(&wc))
    {
        usbi_err(NULL, "can't register hotplug message window class %s", windows_error_str(0));
        goto err_exit;
    }

    // Note: Using HWND_MESSAGE removes broadcast events, like
    // the ones from driverless devices. However the broadcast
    // events you get on driverless provide no data whatsoever
    // about the device, the event (insertion or removal), or
    // even if the device is actually USB. Bummer!
    hHotplugMessage = CreateWindowExA(0, LIBUSB_MSG_WINDOW_CLASS, NULL, 0, 0, 0, 0, 0, HWND_MESSAGE, NULL, NULL, NULL);
    if (hHotplugMessage == NULL)
    {
        usbi_err(NULL, "unable to create hotplug message window: %s", windows_error_str(0));
        goto err_exit;
    }

    dev_bdi.dbcc_size = sizeof(DEV_BROADCAST_DEVICEINTERFACE_A);
    dev_bdi.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;

    // Instead of registering specifically for the GUID_DEVINTERFACE_USB_DEVICE class
    // we will register to receive notifications for all interface classes. This is
    // the only way we will be able to tell if an interface to a composite or HID device
    // has changed, since these events won't trigger a notification message to the
    // USB device
    devnotify = pRegisterDeviceNotificationA(hHotplugMessage, &dev_bdi,
                                             DEVICE_NOTIFY_WINDOW_HANDLE | DEVICE_NOTIFY_ALL_INTERFACE_CLASSES);
    if (devnotify == NULL)
    {
        usbi_err(NULL, "failed to register for device interface notification: %s", windows_error_str(0));
        goto err_exit;
    }

    usbi_dbg(NULL, "hotplug thread waiting for messages");

    // Signal that the thread is ready
    hotplug_ready = TRUE;
    ReleaseSemaphore(hotplug_response, 1, NULL);

    // We need to handle the message pump
    while ((msg_ret = GetMessage(&msg, NULL, 0, 0)) != 0)
    {
        if (msg_ret == -1)
        {
            usbi_err(NULL, "GetMessage error: %s", windows_error_str(0));
        }
        else
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
    }

    ret = 0;
    goto cleanup;

err_exit:
    ReleaseSemaphore(hotplug_response, 1, NULL);
cleanup:
    if (devnotify != NULL)
    {
        pUnregisterDeviceNotification(devnotify);
    }
    if (hHotplugMessage != NULL)
    {
        DestroyWindow(hHotplugMessage);
        hHotplugMessage = NULL;
    }
    pUnregisterClassA(LIBUSB_MSG_WINDOW_CLASS, NULL);

    usbi_dbg(NULL, "hotplug thread quitting");

    return ret;
}

int windows_hotplug_init_once()
{
    // Load DLL imports
    if (!init_dlls(NULL)) {
        usbi_err(NULL, "could not resolve DLL functions");
        goto init_exit;
    }

    hotplug_ready = FALSE;
    hotplug_response = CreateSemaphore(NULL, 0, 1, NULL);
    if (hotplug_response == NULL) {
        usbi_err(NULL, "could not create hotplug response semaphore - aborting");
        goto init_exit;
    }
    hotplug_thread = (HANDLE)_beginthreadex(NULL, 0, windows_hotplug_threaded, NULL, 0, NULL);
    if (hotplug_thread == NULL) {
        usbi_err(NULL, "Unable to create hotplug thread - aborting");
        goto init_exit;
    }
    SetThreadAffinityMask(hotplug_thread, 0);

    // Wait for hotplug thread to init before continuing.
    if (WaitForSingleObject(hotplug_response, INFINITE) != WAIT_OBJECT_0) {
        usbi_err(NULL, "failed to wait for hotplug thread to become ready - aborting");
        goto init_exit;
    }
    if (hotplug_ready != TRUE) {
        usbi_err(NULL, "hotplug thread not ready - aborting");
        goto init_exit;
    }

    return LIBUSB_SUCCESS;

init_exit: // Holds semaphore here.
    if (hotplug_thread) {
        PostMessage(hHotplugMessage, WM_QUIT, 0, 0);
        if (WAIT_OBJECT_0 != WaitForSingleObject(hotplug_thread, INFINITE)) {
            usbi_warn(NULL, "could not wait for hotplug thread to quit");
            TerminateThread(hotplug_thread, 1); // destroy it
        }
        CloseHandle(hotplug_thread);
        hotplug_thread = NULL;
    }
    if (hotplug_response) {
        CloseHandle(hotplug_response);
        hotplug_response = NULL;
    }
    return LIBUSB_ERROR_OTHER;
}

void windows_hotplug_deinit_once()
{
    if (hHotplugMessage) {
        // signal thread to exit
        PostMessage(hHotplugMessage, WM_QUIT, 0, 0);
        if (WAIT_OBJECT_0 != WaitForSingleObject(hotplug_thread, INFINITE)) {
            usbi_dbg(NULL, "could not wait for hotplug thread to quit");
            TerminateThread(hotplug_thread, 1);
        }
        CloseHandle(hotplug_thread);
        hotplug_thread = NULL;
        hHotplugMessage = NULL;
    }
    if (hotplug_response) {
        CloseHandle(hotplug_response);
        hotplug_response = NULL;
    }
    exit_dlls();
}