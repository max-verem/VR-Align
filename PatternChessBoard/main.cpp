/*
    do:
        git submodule update --init --recursive

    for submodules and submodules of submodules...
*/
#include <stdio.h>

#include "instance.h"

#define WS_VER_MAJOR 2
#define WS_VER_MINOR 2
#pragma comment(lib, "ws2_32.lib")

#include <gdiplus.h>
using namespace Gdiplus;
#pragma comment (lib,"Gdiplus.lib")

static volatile int done = 0;


static int key_cmd(int c, instance_h* instance)
{
    switch (c)
    {
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


#if 0
    if(instance->hWnd)
        InvalidateRect(instance->hWnd, NULL, TRUE);
        //RedrawWindow(instance->hWnd, 0, 0, RDW_INVALIDATE | RDW_UPDATENOW);
        //UpdateWindow(instance->hWnd);
#endif

static char console_keypress()
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

    while (!done)
    {
        int c = console_keypress();
        if (c)
            key_cmd(c, instance);
        Sleep(10);

//        if (instance->hWnd)
//            InvalidateRect(instance->hWnd, NULL, TRUE);
    }

    return 0;
};

static VOID OnPaint(HDC hdc, instance_h* instance)
{
    if (!instance)
        return;

    Graphics graphics(hdc);
    graphics.SetSmoothingMode(Gdiplus::SmoothingModeHighQuality);

    SolidBrush
        brush_white(Color(255, 255, 255, 255)),
        brush_black(Color(255, 0, 0, 0));
//        brush_green(Color(255, 0, 80, 0)); // ARGB

    graphics.SetPageUnit(Gdiplus::UnitPixel);
    graphics.SetSmoothingMode(Gdiplus::SmoothingModeNone);

    graphics.FillRectangle(&brush_black, 0, 0, (int)instance->window_size[0], (int)instance->window_size[1]);
    graphics.FillRectangle(&brush_white,
        instance->chess_board_offset[0] - instance->chess_board_step / 2,
        instance->chess_board_offset[1] - instance->chess_board_step / 2,
        (instance->chess_board_dim[0] + 1) * instance->chess_board_step,
        (instance->chess_board_dim[1] + 1) * instance->chess_board_step
    );

    for (int j = 0; j < instance->chess_board_dim[1]; j++)
        for (int i = 0; i < instance->chess_board_dim[0]; i++)
        {
            Rect rect
            (
                instance->chess_board_offset[0] + i * instance->chess_board_step,
                instance->chess_board_offset[1] + j * instance->chess_board_step,
                instance->chess_board_step, instance->chess_board_step
            );

            graphics.FillRectangle(((i + j) & 1) ? &brush_white : &brush_black, rect);
        };

#if 0
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
#endif
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

    instance->window_pos[0] = 800;
    instance->window_pos[1] = 400;

    instance->window_size[0] = 1000;
    instance->window_size[1] = 500;

    instance->chess_board_offset[0] = 100;
    instance->chess_board_offset[1] = 50;

    instance->chess_board_dim[0] = 10;
    instance->chess_board_dim[1] = 7;

    instance->chess_board_step = 60;

#define ARG_CHECK_BEGIN(PARAM_NAME, PARAMS_CNT) if (!strcmp("--" PARAM_NAME, argv[i]) && (PARAMS_CNT + 3) < argc) {
#define ARG_CHECK_END(PARAMS_CNT) i += PARAMS_CNT + 1; }

    for (i = 1; i < argc;)
    {
#if 0
        if (!strcmp("--camera_arm", argv[i]) && (i + 3) < argc)
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
        else
#endif
        ARG_CHECK_BEGIN("window-pos", 2)
            instance->window_pos[0] = atoi(argv[i + 1]);
            instance->window_pos[1] = atoi(argv[i + 2]);
        ARG_CHECK_END(2)
        else
        ARG_CHECK_BEGIN("window-size", 2)
            instance->window_size[0] = atoi(argv[i + 1]);
            instance->window_size[1] = atoi(argv[i + 2]);
        ARG_CHECK_END(2)
        else
        ARG_CHECK_BEGIN("chess-board-offset", 2)
            instance->chess_board_offset[0] = atoi(argv[i + 1]);
            instance->chess_board_offset[1] = atoi(argv[i + 2]);
        ARG_CHECK_END(2)
        else
        ARG_CHECK_BEGIN("chess-board-dim", 2)
            instance->chess_board_dim[0] = atoi(argv[i + 1]);
            instance->chess_board_dim[1] = atoi(argv[i + 2]);
        ARG_CHECK_END(2)
        else
        ARG_CHECK_BEGIN("chess-board-step", 1)
            instance->chess_board_step = atoi(argv[i + 1]);
        ARG_CHECK_END(1)
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
