#include "SocketTransceiver.hpp"

int main()
{
    /*
    
    // For server side:
    SocketTransceiver server(54321);
    if (!server.init())
    {
        std::cout << "socket init failed" << std::endl;
    }
    server.listen(); // blocking
    
    */
    
    
    
    /*
    
    // For client side:
    SocketTransceiver client(54322); // Notice they use two different ports. This is our 'address'
    if (!client.init())
    {
        std::cout << "socket init failed" << std::endl;
    }
    uint8_t data[5] = {'h', 'e', 'l', 'l', 'o'};
    client.send(data, 5, 54321); // send to server port (it's 'address').

    */
    return 0;
}