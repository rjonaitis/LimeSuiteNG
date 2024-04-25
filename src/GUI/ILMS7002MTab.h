#pragma once

#include <wx/panel.h>
#include <map>
#include <cstdint>
#include "commonWxForwardDeclarations.h"
#include "limesuiteng/LMS7002MCSR.h"

namespace lime {
class LMS7002M;
}

class ILMS7002MTab : public wxPanel
{
  public:
    ILMS7002MTab(wxWindow* parent,
        wxWindowID id = wxID_ANY,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = wxTAB_TRAVERSAL);
    virtual void Initialize(lime::LMS7002M* pControl);
    virtual void UpdateGUI();
    void SetChannel(uint8_t channel);

  protected:
    int LMS_ReadLMSReg(lime::LMS7002M* lms, uint16_t address, uint16_t* value);
    int LMS_WriteLMSReg(lime::LMS7002M* lms, uint16_t address, uint16_t value);
    virtual void ParameterChangeHandler(wxCommandEvent& event);
    virtual void SpinParameterChangeHandler(wxSpinEvent& event);

    virtual void WriteParam(const lime::LMS7002MCSR param, uint16_t val);
    virtual int ReadParam(const lime::LMS7002MCSR param);
    int LMS_ReadParam(lime::LMS7002M* lmsControl, const lime::LMS7002MCSR param, uint16_t* value);

    lime::LMS7002M* lmsControl;
    std::map<wxWindow*, lime::LMS7002MCSR> wndId2Enum;
    uint8_t mChannel;
};
