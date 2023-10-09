#ifndef PNL_LIMESDR_H
#define PNL_LIMESDR_H

#include "limesuite/SDRDevice.h"
#include <wx/panel.h>

class wxFlexGridSizer;
class wxCheckBox;

namespace lime
{
class IConnection;
}

class pnlGPIO;

class pnlLimeSDR : public wxPanel
{
public:
    pnlLimeSDR(wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxDefaultSize, int style = 0, wxString name = wxEmptyString);
    void Initialize(lime::SDRDevice* pControl);
    virtual ~pnlLimeSDR();
    virtual void UpdatePanel();
    void OnGPIOChange(wxCommandEvent &event);
    void OnReadAll(wxCommandEvent &event);
    void OnWriteAll(wxCommandEvent &event);

protected:
    int LMS_WriteFPGAReg(lime::SDRDevice *device, uint32_t address, uint16_t val);
    int LMS_ReadFPGAReg(lime::SDRDevice *device, uint32_t address, uint16_t *val);

    pnlGPIO* pnl_gpio;
    wxCheckBox* chkRFLB_A_EN;
    wxCheckBox* chkRFLB_B_EN;
    wxCheckBox* chkTX1_2_LB_SH;
    wxCheckBox* chkTX1_2_LB_AT;
    wxCheckBox* chkTX2_2_LB_SH;
    wxCheckBox* chkTX2_2_LB_AT;
    wxFlexGridSizer* controlsSizer;
    wxFlexGridSizer* mainSizer;

    int chipSelect;
    lime::SDRDevice *device;
    
    DECLARE_EVENT_TABLE()
};

#endif
