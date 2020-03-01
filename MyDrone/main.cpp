//
//  main.cpp
//  MyDrone
//
//  Created by kangzhiyong on 2020/2/29.
//  Copyright © 2020 kangzhiyong. All rights reserved.
//

#include <iostream>
#include "mavlink_connection.hpp"
#include "flyer.hpp"

int main(int argc, const char * argv[]) {
    // insert code here...
    std::cout << "Hello, World!\n";
    MavlinkConnection *conn = new MavlinkConnection("0.0.0.0:14550", true, true);
    Flyer drone(conn);
    drone.start();
    return 0;
}
