#ifndef H_ARESP_MANAGER
#define H_ARESP_MANAGER

#include <string>

class ARespManager
{
public:
    virtual void removeResp(std::string src, uint32_t sequence) = 0;
};

#endif // H_ARESP_MANAGER