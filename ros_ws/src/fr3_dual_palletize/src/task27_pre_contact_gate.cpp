// Independent Task27 pre-contact entry point.
//
// This deliberately reuses the original Task27 scene-facing executor and
// geometry, but compiles a one-shot gate that stops at HIGH_PRE_CONTACT.  It
// never enters OUTER_DESCENT / CONTACT and never publishes a suction command.
#define TASK27_FIVE_CUBE 1
#define TASK27_PRECONTACT_ONLY 1
#include "task26_truck_box_push_in.cpp"
