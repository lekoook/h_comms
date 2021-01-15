#ifndef H_AREQ_MANAGER
#define H_AREQ_MANAGER

#include <cstdint>

class AReqManager
{
public:
    virtual void removeReq(uint32_t sequence, uint16_t entryId, std::string reqTarget) = 0;
    virtual void requeueReq(uint32_t sequence, uint16_t entryId, std::string reqTarget) = 0;
};

#endif // H_AREQ_MANAGER