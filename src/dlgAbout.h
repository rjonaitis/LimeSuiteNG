#ifndef __dlgAbout__
#define __dlgAbout__

/**
@file
Subclass of dlgAbout_view, which is generated by wxFormBuilder.
*/

#include "lms7suiteApp_gui.h"

//// end generated include

/** Implementing dlgAbout_view */
class dlgAbout : public dlgAbout_view
{
  protected:
    // Handlers for dlgAbout_view events.
    void OnInit(wxInitDialogEvent& event);
    void OnbtnClose(wxCommandEvent& event);

  public:
    /** Constructor */
    dlgAbout(wxWindow* parent);
    //// end generated class members
};

#endif // __dlgAbout__
