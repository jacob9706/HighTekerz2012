#ifndef VISION_PROCESSOR_BRIDGE_H
#define VISION_PROCESSOR_BRIDGE_H

struct vspMessage {
	char buf[1024];
	uint32_t count;
	SEM_ID semVSPMessage;
};

vspMessage * getVSPMessage();

#endif
