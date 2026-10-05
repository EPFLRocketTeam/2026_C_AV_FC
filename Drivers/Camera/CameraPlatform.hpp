
#include <cstdint>
#include <cstddef>

bool     cameraPollMessage ();
void     cameraSendMessage (uint16_t messageId, uint8_t length, const uint8_t* data);
uint32_t cameraGetTick ();

void cameraSetup ();
void cameraTick  ();
