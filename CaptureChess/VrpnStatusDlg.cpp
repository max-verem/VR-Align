// VrpnStatusDlg.cpp : implementation file
//

#include "stdafx.h"
#include "VrpnStatusDlg.h"
#include "afxdialogex.h"

#pragma comment(lib, "quatlib.lib")
#pragma comment(lib, "vrpn.lib")

// CVrpnStatusDlg dialog

IMPLEMENT_DYNAMIC(CVrpnStatusDlg, CDialog)

CVrpnStatusDlg::CVrpnStatusDlg(CWnd* pParent /*=nullptr*/)
	: CDialog(IDD_VRPN_STATUS_DLG, pParent)
{
    Create(IDD_VRPN_STATUS_DLG, pParent);
//    ModifyStyleEx(0, WS_EX_CONTROLPARENT);
    vrpn_trk = NULL;
    vrpn_addr = NULL;
}

CVrpnStatusDlg::~CVrpnStatusDlg()
{
    if (vrpn_addr)
        free(vrpn_addr);

    if (vrpn_trk)
        delete vrpn_trk;
}

void CVrpnStatusDlg::set_pos_rot(q_vec_type pos, q_type quat)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    q_vec_copy(curr_pos, pos);
    q_copy(curr_quat, quat);
}

void CVrpnStatusDlg::get_pos_rot(q_vec_type pos, q_type quat)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    q_vec_copy(pos, curr_pos);
    q_copy(quat, curr_quat);
}

static void VRPN_CALLBACK vrpn_handle_tracker(void* userData, const vrpn_TRACKERCB t)
{
    CVrpnStatusDlg* dlg = (CVrpnStatusDlg*)userData;
    q_vec_type pos;
    q_type quat;

    pos[0] = t.pos[0]; pos[1] = t.pos[1]; pos[2] = t.pos[2];
    quat[0] = t.quat[0]; quat[1] = t.quat[1]; quat[2] = t.quat[2]; quat[3] = t.quat[3];

    dlg->set_pos_rot(pos, quat);
};

void CVrpnStatusDlg::setVRPN(const char* addr)
{
    vrpn_addr = strdup(addr);
    vrpn_trk = new vrpn_Tracker_Remote(addr);
    vrpn_trk->register_change_handler(this, vrpn_handle_tracker);
    ((CStatic*)GetDlgItem(IDC_STATIC_FRAME_TITLE))->SetWindowText(addr);
}

void CVrpnStatusDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

LRESULT CVrpnStatusDlg::OnVrpnMainLoop(WPARAM wParam, LPARAM lParam)
{
    if (vrpn_trk)
        vrpn_trk->mainloop();

    return 0;
}

LRESULT CVrpnStatusDlg::OnVrpnUpdate(WPARAM wParam, LPARAM lParam)
{
    q_type quat;
    q_vec_type pos, yawPitchRoll;
    char buf[1024];

    get_pos_rot(pos, quat);

    q_to_euler(yawPitchRoll, quat);

    /*
            resulting vector is:
            [0] - Q_YAW - rotation about Z
            [1] - Q_PITCH - rotation about Y
            [2] - Q_ROLL - rotation about X
    */

    snprintf(buf, sizeof(buf), "X=%8.4f, Y=%8.4f, Z=%8.4f", pos[0], pos[1], pos[2]);
    ((CStatic*)GetDlgItem(IDC_STATIC_VRPN_POSITION))->SetWindowText(buf);

    snprintf(buf, sizeof(buf), "Yaw/Z=%8.4f, Pitch/Y=%8.4f, Roll/X=%8.4f",
        yawPitchRoll[0] * 180.0 / 3.1415926,
        yawPitchRoll[1] * 180.0 / 3.1415926,
        yawPitchRoll[2] * 180.0 / 3.1415926);
    ((CStatic*)GetDlgItem(IDC_STATIC_VRPN_ROTATION))->SetWindowText(buf);

    return 0;
}


BEGIN_MESSAGE_MAP(CVrpnStatusDlg, CDialog)
    ON_MESSAGE(WM_UPDATE_VRPN_MESSAGE, &CVrpnStatusDlg::OnVrpnUpdate)
    ON_MESSAGE(WM_MAIN_LOOP_VRPN_MESSAGE, &CVrpnStatusDlg::OnVrpnMainLoop)
END_MESSAGE_MAP()


// CVrpnStatusDlg message handlers
