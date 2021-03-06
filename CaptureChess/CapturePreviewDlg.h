/* -LICENSE-START-
** Copyright (c) 2020 Blackmagic Design
**  
** Permission is hereby granted, free of charge, to any person or organization 
** obtaining a copy of the software and accompanying documentation (the 
** "Software") to use, reproduce, display, distribute, sub-license, execute, 
** and transmit the Software, and to prepare derivative works of the Software, 
** and to permit third-parties to whom the Software is furnished to do so, in 
** accordance with:
** 
** (1) if the Software is obtained from Blackmagic Design, the End User License 
** Agreement for the Software Development Kit (“EULA”) available at 
** https://www.blackmagicdesign.com/EULA/DeckLinkSDK; or
** 
** (2) if the Software is obtained from any third party, such licensing terms 
** as notified by that third party,
** 
** and all subject to the following:
** 
** (3) the copyright notices in the Software and this entire statement, 
** including the above license grant, this restriction and the following 
** disclaimer, must be included in all copies of the Software, in whole or in 
** part, and all derivative works of the Software, unless such copies or 
** derivative works are solely in the form of machine-executable object code 
** generated by a source language processor.
** 
** (4) THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS 
** OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
** FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT 
** SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE 
** FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE, 
** ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
** DEALINGS IN THE SOFTWARE.
** 
** A copy of the Software is available free of charge at 
** https://www.blackmagicdesign.com/desktopvideo_sdk under the EULA.
** 
** -LICENSE-END-
*/

#pragma once

#include <map>
#include <mutex>
#include "resource.h"
#include "DeckLinkAPI_h.h"

#include "DeckLinkDeviceDiscovery.h"
#include "DeckLinkDevice.h"
#include "PreviewWindow.h"
#include "ProfileCallback.h"
#include "VrpnStatusDlg.h"

// Custom messages
#define WM_REFRESH_INPUT_STREAM_DATA_MESSAGE	(WM_APP + 1)
#define WM_DETECT_VIDEO_MODE_MESSAGE			(WM_APP + 2)
#define WM_ADD_DEVICE_MESSAGE					(WM_APP + 3)
#define WM_REMOVE_DEVICE_MESSAGE				(WM_APP + 4)
#define WM_ERROR_RESTARTING_CAPTURE_MESSAGE		(WM_APP + 5)
#define WM_UPDATE_PROFILE_MESSAGE				(WM_APP + 6)

#define VRPN_STATUS_MAX 4

class CCapturePreviewDlg : public CDialog
{
public:
	CCapturePreviewDlg(CWnd* pParent = NULL);

	// Dialog Data
	enum { IDD = IDD_CAPTUREPREVIEW_DIALOG };
	
	// UI-related handlers
	afx_msg void			OnNewDeviceSelected();
	afx_msg void			OnInputConnectionSelected();
	afx_msg void			OnStartStopBnClicked();
	afx_msg void			OnAutoDetectCBClicked();
	afx_msg void			OnClose();

	// Custom message handlers
	afx_msg LRESULT			OnRefreshInputStreamData(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT			OnDetectVideoMode(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT			OnAddDevice(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT			OnRemoveDevice(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT			OnErrorRestartingCapture(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT			OnProfileUpdate(WPARAM wParam, LPARAM lParam);

    // my controls
    afx_msg void OnSelectCaptureDirBnClicked();
    afx_msg void OnGrabFrameBnClicked();

	// DeckLinkDevice delegate methods
	void					ShowErrorMessage(TCHAR* msg, TCHAR* title);

protected:
	// Internal helper methods
	void					EnableInterface(bool enabled);
	void					RefreshInputDeviceList();
	void					RefreshInputConnectionList();
	void					RefreshVideoModeList();
	void					StartCapture();
	void					StopCapture();
	void					AddDevice(CComPtr<IDeckLink>& deckLink);
	void					RemoveDevice(CComPtr<IDeckLink>& deckLink);
	void					VideoFrameArrived(CComPtr<IDeckLinkVideoInputFrame>& videoFrame);
	void					HaltStreams(CComPtr<IDeckLinkProfile>& newProfile);
	void					HandleDeviceError(DeviceError error);

	// UI elements
	CComboBox				m_deviceListCombo;
	CComboBox				m_inputConnectionCombo;
	CButton					m_applyDetectedInputModeCheckbox;
	CComboBox				m_modeListCombo;
	CButton					m_startStopButton;
	CStatic					m_invalidInputLabel;
	CStatic					m_previewBox;
    CStatic					m_vrpns;
    std::vector<CVrpnStatusDlg*> m_vrpn_status;
    CComPtr<PreviewWindow>	m_previewWindow;

	CSize					m_minDialogSize;

	//
	std::mutex							m_mutex; // to synchronise access to the above structures
	CComPtr<DeckLinkDevice>				m_selectedDevice;
	BMDVideoConnection					m_selectedInputConnection;
	CComPtr<DeckLinkDeviceDiscovery>	m_deckLinkDiscovery;
	CComPtr<ProfileCallback>			m_profileCallback;
    time_t save_flag;

	std::map<IDeckLink*, CComPtr<DeckLinkDevice>>		m_inputDevices;

	//
	virtual void			DoDataExchange(CDataExchange* pDX);	// DDX/DDV support
	HICON					m_hIcon;

	// Generated message map functions
	virtual BOOL			OnInitDialog();
	afx_msg void			OnPaint();
	afx_msg HCURSOR			OnQueryDragIcon();
	afx_msg void			OnGetMinMaxInfo(MINMAXINFO* minMaxInfo);
    afx_msg void OnTimer(UINT_PTR nIdEvent);
	DECLARE_MESSAGE_MAP()
};
