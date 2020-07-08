#ifndef CONFIG_H_
#define CONFIG_H_

#include <algorithm>
#include <stdint.h>

/* RFC3561: section 10 */

const uint16_t ACTIVE_ROUTE_TIMEOUT = 3000; // ms // If Hello messages are used, then the ACTIVE_ROUTE_TIMEOUT parameter value MUST be more than the value (ALLOWED_HELLO_LOSS * HELLO_INTERVAL).
const uint8_t ALLOWED_HELLO_LOSS = 2;
const uint16_t HELLO_INTERVAL = 1000; // ms
const uint8_t LOCAL_ADD_TTL = 2;
const uint8_t MIN_REPAIR_TTL = 0; // TODO // should be the last known hop count to the destination.
const uint8_t NET_DIAMETER = 35;
const uint8_t NODE_TRAVERSAL_TIME = 40; // ms
const uint8_t RERR_RATELIMIT = 10;
const uint8_t RREQ_RETRIES = 2;
const uint8_t RREQ_RATELIMIT = 10;
const uint8_t TIMEOUT_BUFFER = 2;
const uint8_t TTL_START = 1;
const uint8_t TTL_INCREMENT = 2;
const uint8_t TTL_THRESHOLD = 7;
const uint8_t TTL_VALUE = 0; // TODO // the value of the TTL field in the IP header while the expanding ring search is being performed.
const uint8_t K = 5; // K = 5 is recommended.

// Derived constants
const uint16_t DELETE_PERIOD = K * std::max(ACTIVE_ROUTE_TIMEOUT, HELLO_INTERVAL);
const uint8_t MAX_REPAIR_TTL = 0.3 * NET_DIAMETER;
const uint16_t MY_ROUTE_TIMEOUT = 2 * ACTIVE_ROUTE_TIMEOUT;
const uint16_t NET_TRAVERSAL_TIME = 2 * NODE_TRAVERSAL_TIME * NET_DIAMETER;
const uint8_t NEXT_HOP_WAIT = NODE_TRAVERSAL_TIME + 10;
const uint16_t PATH_DISCOVERY_TIME = 2 * NET_TRAVERSAL_TIME;
const uint8_t RING_TRAVERSAL_TIME = 2 * NODE_TRAVERSAL_TIME * (TTL_VALUE + TIMEOUT_BUFFER);
const uint16_t BLACKLIST_TIMEOUT = RREQ_RETRIES * NET_TRAVERSAL_TIME;

#endif // CONFIG_H_
