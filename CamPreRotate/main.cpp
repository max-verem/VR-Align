/*
    do:
        git submodule update --init --recursive

    for submodules and submodules of submodules...
*/

#include "instance.h"

#define WS_VER_MAJOR 2
#define WS_VER_MINOR 2
#pragma comment(lib, "ws2_32.lib")

#pragma comment(lib, "quatlib.lib")
#pragma comment(lib, "vrpn.lib")

#include <gdiplus.h>
using namespace Gdiplus;
#pragma comment (lib,"Gdiplus.lib")

static volatile int done = 0;


static int key_cmd(int c, instance_h* instance)
{
    switch (c)
    {
        case 'q': instance->prerot[0] -= 0.1; break;
        case 'w': instance->prerot[0] -= 0.01; break;
        case 'e': instance->prerot[0] += 0.01; break;
        case 'r': instance->prerot[0] += 0.1; break;
        case 't': instance->prerot[0] = 0.0; break;

        case 'a': instance->prerot[1] -= 0.1; break;
        case 's': instance->prerot[1] -= 0.01; break;
        case 'd': instance->prerot[1] += 0.01; break;
        case 'f': instance->prerot[1] += 0.1; break;
        case 'g': instance->prerot[1] = 0.0; break;

        case 'z': instance->prerot[2] -= 0.1; break;
        case 'x': instance->prerot[2] -= 0.01; break;
        case 'c': instance->prerot[2] += 0.01; break;
        case 'v': instance->prerot[2] += 0.1; break;
        case 'b': instance->prerot[2] = 0.0; break;

        default:
            fprintf(stderr, "\nhere [%c]\n", c);
    }

    return 0;
}


// install a signal handler to shut down the devices
// On Windows, the signal handler is run in a different thread from
// the main application.  We don't want to go destroying things in
// here while they are being used there, so we set a flag telling the
// main program it is time to exit.
#if defined(_WIN32) && !defined(__CYGWIN__)
/**
* Handle exiting cleanly when we get ^C or other signals.
*/
BOOL WINAPI handleConsoleSignalsWin(DWORD signaltype)
{
    switch (signaltype) {
    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        done = 1;
        return TRUE;
        // Don't exit, but return FALSE so default handler
        // gets called. The default handler, ExitProcess, will exit.
    default:
        return FALSE;
    }
}
#endif

static void calc_v_plane_intercept(const q_vec_type A, const q_vec_type B, q_vec_type O, double deep)
{
    double t = (deep - A[0]) / (B[0] - A[0]);

    O[0] = deep;
    O[1] = A[1] + t * (B[1] - A[1]);
    O[2] = A[2] + t * (B[2] - A[2]);
};

static void calc_v_plane_proj_px(const q_vec_type LU, const q_vec_type V, q_vec_type P, double px_per_meter)
{
    P[0] = V[1] - LU[1];
    P[1] = LU[2] - V[2];
    P[2] = 0;

    P[0] *= px_per_meter;
    P[1] *= px_per_meter;
}


static void VRPN_CALLBACK vrpn_handle_tracker(void* userData, const vrpn_TRACKERCB t)
{
    int i;
    q_type arm_quat, prerot_quat;
    
    instance_h* instance = (instance_h*)userData;

    /* create prerotation quaterion */
    q_from_euler(prerot_quat,
        instance->prerot[0] * 3.1415926 / 180.0,
        instance->prerot[1] * 3.1415926 / 180.0,
        instance->prerot[2] * 3.1415926 / 180.0); // Yaw/Pitch/Roll

    // rotate arm vectors
    q_invert(arm_quat, t.quat);

    // calc arms positions
    for (i = 0; i < ARM_LAST_POS; i++)
    {
        q_vec_type arm_vec, arm_vec_rot, arm_vec_rot2;

        q_vec_copy(arm_vec, instance->camera_arm);

        /* for non O point we add deep 20 meters */
        if (i)
            arm_vec[0] += instance->frustum_deep;

        /* shift vectors for L/R/U/D points */
        switch (i)
        {
            case ARM_L_POS:
                arm_vec[1] -= instance->frustum_radius;
                break;

            case ARM_R_POS:
                arm_vec[1] += instance->frustum_radius;
                break;

            case ARM_U_POS:
                arm_vec[2] += instance->frustum_radius;
                break;

            case ARM_D_POS:
                arm_vec[2] -= instance->frustum_radius;
                break;
        };

        // rotate arm vectors
        q_xform(arm_vec_rot, arm_quat, arm_vec);
        q_xform(arm_vec_rot2, prerot_quat, arm_vec_rot);

        /* calc position of A / B arms */
        q_vec_add(instance->armX_poses[i], t.pos, arm_vec_rot2);
    };

    // calc intercepts
    for (i = ARM_C_POS; i < ARM_LAST_POS; i++)
        calc_v_plane_intercept(instance->armX_poses[ARM_O_POS], instance->armX_poses[i], instance->armX_intercepts[i], instance->v_screen_pos[0]);

    // calc intercepts
    for (i = ARM_C_POS; i < ARM_LAST_POS; i++)
        calc_v_plane_intercept(instance->armX_poses[ARM_O_POS], instance->armX_poses[i], instance->armX_intercepts[i], instance->v_screen_pos[0]);

    // calc projections
    for (i = ARM_C_POS; i < ARM_LAST_POS; i++)
        calc_v_plane_proj_px(instance->v_screen_pos, instance->armX_intercepts[i], instance->armX_proj_px[i],
            instance->window_size[0] / instance->v_screen_size[1]);

    fprintf(stdout, "armO_pos=[%7.3f,%7.3f,%7.3f],[%6.3f, %6.3f, %6.3f],[%6.1f,%6.1f,%6.1f],[Y=%7.2f,R=%7.2f,P=%7.2f]\r",
        instance->armX_poses[ARM_O_POS][0], instance->armX_poses[ARM_O_POS][1], instance->armX_poses[ARM_O_POS][2],
        instance->armX_intercepts[ARM_C_POS][0], instance->armX_intercepts[ARM_C_POS][1], instance->armX_intercepts[ARM_C_POS][2],
        instance->armX_proj_px[ARM_C_POS][0], instance->armX_proj_px[ARM_C_POS][1], instance->armX_proj_px[ARM_C_POS][2],
        instance->prerot[0], instance->prerot[1], instance->prerot[2]);

    if(instance->hWnd)
        InvalidateRect(instance->hWnd, NULL, TRUE);
        //RedrawWindow(instance->hWnd, 0, 0, RDW_INVALIDATE | RDW_UPDATENOW);
        //UpdateWindow(instance->hWnd);
};

char console_keypress()
{
    DWORD ne = 0;
    HANDLE hStdin = GetStdHandle(STD_INPUT_HANDLE);
    if (GetNumberOfConsoleInputEvents(hStdin, &ne) && ne)
    {
        INPUT_RECORD irInBuf[128];
        DWORD cNumRead;
        if (ReadConsoleInput(
            hStdin,      // input buffer handle 
            irInBuf,     // buffer to read into 
            128,         // size of read buffer 
            &cNumRead)) // number of records read 
        {
            if (cNumRead && irInBuf[0].EventType == KEY_EVENT && irInBuf[0].Event.KeyEvent.bKeyDown)
                return irInBuf[0].Event.KeyEvent.uChar.AsciiChar;
        }
    }
    return 0;
}


static unsigned long WINAPI vrpn_thread(void* ptr)
{
    instance_h* instance = (instance_h*)ptr;

    instance->vrpn_tkr = new vrpn_Tracker_Remote(instance->vrpn_address);
    instance->vrpn_tkr->register_change_handler(instance, vrpn_handle_tracker);

    while (!done)
    {
        int c = console_keypress();
        if (c)
            key_cmd(c, instance);
        instance->vrpn_tkr->mainloop();
    }
        

    delete instance->vrpn_tkr;

    return 0;
};

static VOID OnPaint(HDC hdc, instance_h* instance)
{
    if (!instance)
        return;

    Graphics graphics(hdc);
    graphics.SetSmoothingMode(Gdiplus::SmoothingModeHighQuality);
    Pen pen_blue(Color(255, 0, 0, 255), 2), pen_green(Color(255, 255, 0, 255), 2);

    graphics.DrawLine(&pen_blue,
        (REAL)instance->armX_proj_px[ARM_L_POS][0], (REAL)instance->armX_proj_px[ARM_L_POS][1],
        (REAL)instance->armX_proj_px[ARM_R_POS][0], (REAL)instance->armX_proj_px[ARM_R_POS][1]);

    graphics.DrawLine(&pen_blue,
        (REAL)instance->armX_proj_px[ARM_U_POS][0], (REAL)instance->armX_proj_px[ARM_U_POS][1],
        (REAL)instance->armX_proj_px[ARM_D_POS][0], (REAL)instance->armX_proj_px[ARM_D_POS][1]);

    graphics.DrawLine(&pen_green,
        0.0, (REAL)instance->window_size[1] / 2.0,
        (REAL)instance->window_size[0], (REAL)instance->window_size[1] / 2.0);

    graphics.DrawLine(&pen_green,
        (REAL)instance->window_size[0] / 2.0, 0.0,
        (REAL)instance->window_size[0] / 2.0, (REAL)instance->window_size[1]);
}

static LRESULT CALLBACK WndProc(HWND hWnd, UINT message,
    WPARAM wParam, LPARAM lParam)
{
    HGDIOBJ hOld;
    HBITMAP hbmMem;
    PAINTSTRUCT ps;
    HDC hdc, hdcMem;

    instance_h* instance = (instance_h*)GetWindowLongPtr(hWnd, GWLP_USERDATA);

    switch (message)
    {
        case WM_PAINT:

            if (!instance)
                return 0;

            hdc = BeginPaint(hWnd, &ps);

            // Create an off-screen DC for double-buffering
            hdcMem = CreateCompatibleDC(hdc);
            hbmMem = CreateCompatibleBitmap(hdc, instance->window_size[0], instance->window_size[1]);
            hOld = SelectObject(hdcMem, hbmMem);

            OnPaint(hdcMem, instance);

            // Transfer the off-screen DC to the screen
            BitBlt(hdc, 0, 0, instance->window_size[0], instance->window_size[1], hdcMem, 0, 0, SRCCOPY);

            // Free-up the off-screen DC
            SelectObject(hdcMem, hOld);

            DeleteObject(hbmMem);
            DeleteDC(hdcMem);

            EndPaint(hWnd, &ps);
            return 0;

        case WM_ERASEBKGND:
            return 1;

        case WM_CHAR:
            key_cmd((int)wParam, instance);
            return 0;

        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;

        default:
            return DefWindowProc(hWnd, message, wParam, lParam);
    }
} // WndProc

int main(int argc, char** argv)
{
    int i;

    /* allocate instance */
    instance_h* instance = (instance_h*)malloc(sizeof(instance_h));
    memset(instance, 0, sizeof(instance_h));

    /* setup params default */
    instance->lock = CreateMutex(NULL, FALSE, NULL);
    instance->vrpn_address = strdup("virtual/TRACKER-78@10.1.5.83:3885");
    instance->camera_arm[0] = 0.06;
    instance->camera_arm[1] = 0.0;
    instance->camera_arm[2] = -0.16;
    instance->frustum_deep = 20.0;
    instance->frustum_radius = 2.0;
    instance->v_screen_pos[0] = 8.5;
    instance->v_screen_pos[1] = -5.0;
    instance->v_screen_pos[2] = 4.6;
    instance->v_screen_size[0] = 0;
    instance->v_screen_size[1] = 10;
    instance->v_screen_size[2] = 5;
    instance->window_pos[0] = 800;
    instance->window_pos[1] = 400;
    instance->window_size[0] = 1000;
    instance->window_size[1] = 500;
    instance->prerot[0] = instance->prerot[1] = instance->prerot[2] = 0.0;

    for (i = 1; i < argc;)
    {
        if (!strcmp("vrpn_address", argv[i]) && (i + 1) < argc)
        {
            instance->vrpn_address = strdup(argv[i + 1]);
            i += 2;
        }
        else if (!strcmp("frustum_deep", argv[i]) && (i + 1) < argc)
        {
            instance->frustum_deep = atof(argv[i + 1]);
            i += 2;
        }
        else if (!strcmp("frustum_radius", argv[i]) && (i + 1) < argc)
        {
            instance->frustum_radius = atof(argv[i + 1]);
            i += 2;
        }
        else if (!strcmp("prerot", argv[i]) && (i + 3) < argc)
        {
            instance->prerot[0] = atof(argv[i + 1]);
            instance->prerot[1] = atof(argv[i + 2]);
            instance->prerot[2] = atof(argv[i + 3]);
            i += 4;
        }
        else if (!strcmp("camera_arm", argv[i]) && (i + 3) < argc)
        {
            instance->camera_arm[0] = atof(argv[i + 1]);
            instance->camera_arm[1] = atof(argv[i + 2]);
            instance->camera_arm[2] = atof(argv[i + 3]);
            i += 4;
        }
        else if (!strcmp("v_screen_pos", argv[i]) && (i + 3) < argc)
        {
            instance->v_screen_pos[0] = atof(argv[i + 1]);
            instance->v_screen_pos[1] = atof(argv[i + 2]);
            instance->v_screen_pos[2] = atof(argv[i + 3]);
            i += 4;
        }
        else if (!strcmp("v_screen_size", argv[i]) && (i + 3) < argc)
        {
            instance->v_screen_size[0] = atof(argv[i + 1]);
            instance->v_screen_size[1] = atof(argv[i + 2]);
            instance->v_screen_size[2] = atof(argv[i + 3]);
            i += 4;
        }
        else if (!strcmp("window_pos", argv[i]) && (i + 2) < argc)
        {
            instance->window_pos[0] = atof(argv[i + 1]);
            instance->window_pos[1] = atof(argv[i + 2]);
            i += 3;
        }
        else if (!strcmp("window_size", argv[i]) && (i + 2) < argc)
        {
            instance->window_size[0] = atof(argv[i + 1]);
            instance->window_size[1] = atof(argv[i + 2]);
            i += 3;
        }
        else
        {
            fprintf(stderr, "Failed to parse arg #%d [%s]\n", i, argv[i]);
            exit(1);
        };
    };



    // This handles all kinds of signals.
    SetConsoleCtrlHandler(handleConsoleSignalsWin, TRUE);

    /* run vrpn reader thread */
    instance->vrpn_th = CreateThread
    (
        NULL,
        1024,
        vrpn_thread,
        instance,
        0,
        NULL
    );

    // Initialize GDI+.
    GdiplusStartupInput gdiplusStartupInput;
    ULONG_PTR gdiplusToken;
    GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);

    WNDCLASS wndClass;
    memset(&wndClass, 0, sizeof(WNDCLASS));
    wndClass.style = 0; // CS_HREDRAW | CS_VREDRAW;
    wndClass.lpfnWndProc = WndProc;
//    wndClass.cbClsExtra = 0;
//    wndClass.cbWndExtra = 0;
//    wndClass.hInstance = hInstance;
//    wndClass.hIcon = LoadIcon(NULL, IDI_APPLICATION);
//    wndClass.hCursor = LoadCursor(NULL, IDC_ARROW);
    wndClass.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
    wndClass.lpszMenuName = NULL;
    wndClass.lpszClassName = TEXT("GettingStarted");
    RegisterClass(&wndClass);

    instance->hWnd = CreateWindowEx
    (
        0, //WS_EX_APPWINDOW | WS_EX_OVERLAPPEDWINDOW, // DWORD     dwExStyle,
        wndClass.lpszClassName,     // LPCSTR    lpClassName,
        TEXT("lpWindowName"),       // LPCSTR    lpWindowName,
        WS_VISIBLE | WS_POPUP | WS_CLIPSIBLINGS | WS_CLIPCHILDREN, // DWORD     dwStyle,
        instance->window_pos[0],    // int       X,
        instance->window_pos[1],    // int       Y,
        instance->window_size[0],   // int       nWidth,
        instance->window_size[1],   // int       nHeight,
        NULL,                       // HWND      hWndParent,
        NULL,                       // HMENU     hMenu,
        wndClass.hInstance,         // HINSTANCE hInstance,
        NULL                        // LPVOID    lpParam
    );

    if (!instance->hWnd)
    {
        fprintf(stderr, "Failed to create window\n");
        exit(1);
    }

    SetWindowLongPtr(instance->hWnd, GWLP_USERDATA, (LONG_PTR)instance);
    SetWindowPos(instance->hWnd, NULL,
        instance->window_pos[0], instance->window_pos[1],
        instance->window_size[0], instance->window_size[1], 0);
    UpdateWindow(instance->hWnd);

    MSG msg;
    while (GetMessage(&msg, NULL, 0, 0))
    {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
        if (done)
            break;
    }

    GdiplusShutdown(gdiplusToken);

#if 0
    /* main sleep */
    while (!done)
        Sleep(100);
#endif

    return 0;
};
