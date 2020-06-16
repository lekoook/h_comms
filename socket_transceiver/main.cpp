#include "SocketTransceiver.hpp"

int main()
{
    /*
    
    SocketTransceiver server(54321);
    server.listen();
    for (int i = 0; i < 5; i++)
    {
        std::cout << "I'm free to do what I like in the mean time and I do this 5 times." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    server.stopListen(); // should always be called before server gets destroyed.
    
    */
    
    
    
    /*
    
    // For client side:
    SocketTransceiver client(54322); // Notice they use two different ports. This is our 'address'
    uint8_t data[5] = {'h', 'e', 'l', 'l', 'o'};
    client.send(data, 5, 54321); // send to server port (it's 'address').

    */
    return 0;
}