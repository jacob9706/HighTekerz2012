<<<<<<< HEAD
#ifndef VISION_PROCESSOR_BRIDGE_H
#define VISION_PROCESSOR_BRIDGE_H

struct vspMessage {
	char buf[1024];
	uint32_t count;
	SEM_ID semVSPMessage;
};

vspMessage * getVSPMessage();

#endif
=======
#ifndef VISION_PROCESSOR_BRIDGE_H
#define VISION_PROCESSOR_BRIDGE_H

struct vspMessage {
	char buf[1024];
	uint32_t count;
	SEM_ID semVSPMessage;
};

vspMessage * getVSPMessage();

#endif
>>>>>>> 5cf0ff622f058e49bd836e49cbd450d29583cd9d
