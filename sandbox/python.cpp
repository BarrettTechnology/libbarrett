/*
 * python.cpp
 *
 *  Created on: Oct 20, 2011
 *      Author: dc
 */

#include <string>
#include <boost/python.hpp>

#include <barrett/bus/can_socket.h>

#include <barrett/products/product_manager.h>
#include <barrett/products/puck.h>

#include <barrett/systems/wam.h>


using namespace barrett;


BOOST_PYTHON_MODULE(libbtpy)
{
    using namespace boost::python;

    class_<systems::Wam<7>, boost::noncopyable>("Wam7", no_init)
    ;

    class_<ProductManager, boost::noncopyable>("ProductManager")
    	.def("waitForWam", &ProductManager::waitForWam)
    	.def("wakeAllPucks", &ProductManager::wakeAllPucks)
    	.def("foundWam7", &ProductManager::foundWam7)
    	.def("getWam7", &ProductManager::getWam7, return_internal_reference<>())
    ;

    class_<bus::CANSocket, boost::noncopyable>("CANSocket", init<int>())
    	.def("send", &bus::CANSocket::send)
    	.def("receiveRaw", &bus::CANSocket::receiveRaw)
    ;

    class_<Puck>("Puck", init<bus::CANSocket&, int>())
		.def("wake", (void (Puck::*)())&Puck::wake)  // cast to resolve the overload
//    	.def("getProperty", &Puck::getProperty)
    ;
}

