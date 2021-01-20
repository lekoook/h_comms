#ifndef H_CONFIG_PARAMS
#define H_CONFIG_PARAMS

namespace aodv
{

namespace config
{
    int const TIMEOUT_BUFFER           = 2;
    int const TTL_VALUE                = 0;
    int const RREQ_RETRIES             = 2;
    int const ACTIVE_ROUTE_TIMEOUT     = 3000; // milliseconds
    int const ALLOWED_HELLO_LOSS       = 2;
    int const NODE_TRAVERSAL_TIME      = 40; // milliseconds
    int const NET_DIAMETER             = 35;
    int const NET_TRAVERSAL_TIME       = 2 * NODE_TRAVERSAL_TIME * NET_DIAMETER;
    int const BLACKLIST_TIMEOUT        = RREQ_RETRIES * NET_TRAVERSAL_TIME;
    int const HELLO_INTERVAL           = 1000; // milliseconds
    int const DELETE_PERIOD            = ALLOWED_HELLO_LOSS * HELLO_INTERVAL;
    int const LOCAL_ADD_TTL            = 2;
    int const MAX_REPAIR_TTL           = 0.3 * NET_DIAMETER;
    int const MIN_REPAIR_TTL           = 1;
    int const MY_ROUTE_TIMEOUT         = 2 * ACTIVE_ROUTE_TIMEOUT;
    int const NEXT_HOP_WAIT            = NODE_TRAVERSAL_TIME + 10;
    int const PATH_DISCOVERY_TIME      = 2 * NET_TRAVERSAL_TIME;
    int const RERR_RATELIMIT           = 10;
    int const RING_TRAVERSAL_TIME      = 2 * NODE_TRAVERSAL_TIME * (TTL_VALUE + TIMEOUT_BUFFER);
    int const RREQ_RATELIMIT           = 10;
    int const TTL_START                = 1;
    int const TTL_INCREMENT            = 2;
    int const TTL_THRESHOLD            = 7;
}

}

#endif // H_CONFIG_PARAMS