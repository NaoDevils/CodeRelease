#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(NetworkStatus,
  ENUM(Status,
    unconfigured,
    configuring,
    configured,
    failed
  );
  ,
	(unsigned)(0) configureTimestamp,
	(Status)(unconfigured) status
);
