#pragma once

#include "resource.h"

#include <mutex>
#include <quat.h>
#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>

#define WM_UPDATE_VRPN_MESSAGE (WM_APP + 1)
#define WM_MAIN_LOOP_VRPN_MESSAGE (WM_APP + 2)

class CVrpnStatusDlg : public CDialog
{
	DECLARE_DYNAMIC(CVrpnStatusDlg)

    vrpn_Tracker_Remote *vrpn_trk;
    std::mutex m_mutex; // to synchronise access to the above structures
    q_type curr_quat;
    q_vec_type curr_pos;
    char* vrpn_addr;
public:
	CVrpnStatusDlg(CWnd* pParent = nullptr);   // standard constructor
	virtual ~CVrpnStatusDlg();
    void setVRPN(const char* addr);
    void set_pos_rot(q_vec_type pos, q_type quat);
    void get_pos_rot(q_vec_type pos, q_type quat);

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_VRPN_STATUS_DLG };
#endif

    afx_msg LRESULT OnVrpnUpdate(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnVrpnMainLoop(WPARAM wParam, LPARAM lParam);

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
};
