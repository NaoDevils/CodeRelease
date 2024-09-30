#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"

STREAMABLE(USBStatus,
	ENUM(MountStatus,
		inactive,
		mounting,
		unmounting,
		notMounted,
		readWrite,
		readOnly,
		unknown
	);
	,
	(unsigned)(0) mountTimestamp,
	(MountStatus)(MountStatus::inactive) status,
	(std::string)("") path,
	(std::string)("") logPath
);
