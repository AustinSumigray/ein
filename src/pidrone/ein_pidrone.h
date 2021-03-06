#ifndef _EIN_PIDRONE_H_
#define _EIN_PIDRONE_H_

class EinPidroneConfig;
class MachineState;
void robotInitializeConfig(MachineState * ms);
void robotInitializeMachine(MachineState * ms);
void robotEndPointCallback(MachineState * ms);
void robotSetCurrentJointPositions(MachineState * ms);
void robotInitializeSerial(MachineState * ms);

void robotActivateSensorStreaming(MachineState * ms);
void robotDeactivateSensorStreaming(MachineState * ms);
void robotUpdate(MachineState * ms);

#include <vector>
#include <memory>
#endif /* _EIN_PIDRONE_H_ */
