/*
 * python.cpp
 *
 *  Created on: Oct 20, 2011
 *      Author: dc
 */

#include <string>
#include <stdexcept>
#include <cstdlib>  // For mkstmp()
#include <cstdio>  // For remove()

#include <boost/python.hpp>

#include <barrett/bus/can_socket.h>

#include <barrett/products/product_manager.h>
#include <barrett/products/puck.h>

#include <barrett/systems/wam.h>

#include <barrett/log.h>
#include <barrett/units.h>
#include <barrett/systems.h>


using namespace barrett;


class FTSAccel : public systems::System, public systems::SingleOutput<units::CartesianAcceleration::type> {
	BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

public:
	explicit FTSAccel(ForceTorqueSensor* fts_, const std::string& sysName = "FTSAccel") :
		systems::System(sysName), systems::SingleOutput<ca_type>(this), fts(fts_)
	{}
	virtual ~FTSAccel() { mandatoryCleanUp(); }

protected:
	ForceTorqueSensor* fts;

	virtual void operate() {
		fts->updateAccel(true);
		outputValue->setData(&fts->getAccel());
	}
};


void takeAccelSample(ProductManager& pm, int duration_us, const char* fileName)
{
	BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

	if ( !pm.foundForceTorqueSensor() ) {
		throw std::runtime_error("Couldn't find an FTS!");
	}
	pm.startExecutionManager();

	char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		throw std::runtime_error("Couldn't create temporary file!");
	}


	systems::Ramp time(pm.getExecutionManager(), 1.0);
	FTSAccel ftsa(pm.getForceTorqueSensor());

	systems::TupleGrouper<double, ca_type> tg;
	connect(time.output, tg.getInput<0>());
	connect(ftsa.output, tg.getInput<1>());

	typedef boost::tuple<double, ca_type> tuple_type;
	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type> logger(
			pm.getExecutionManager(),
			new log::RealTimeWriter<tuple_type>(tmpFile, PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);

	time.start();
	connect(tg.output, logger.input);
	printf("Logging started.\n");


	usleep(duration_us);


	logger.closeLog();
	printf("Logging stopped.\n");

	log::Reader<tuple_type> lr(tmpFile);
	lr.exportCSV(fileName);
	printf("Output written to %s.\n", fileName);
	std::remove(tmpFile);
}

bool isWamActivated(ProductManager& pm) {
	return pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE;
}


BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Wam4_gravityCompensate_overloads, gravityCompensate, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Wam7_gravityCompensate_overloads, gravityCompensate, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ProductManager_getExecutionManager_overloads, getExecutionManager, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ProductManager_waitForWam_overloads, waitForWam, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ProductManager_getWam4_overloads, getWam4, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ProductManager_getWam7_overloads, getWam7, 0, 2)

BOOST_PYTHON_MODULE(libbtpy)
{
    using namespace boost::python;

    class_<systems::RealTimeExecutionManager, boost::noncopyable>("RealTimeExecutionManager", no_init)
    ;

    class_<systems::Wam<4>, boost::noncopyable>("Wam4", no_init)
    	.def("gravityCompensate", &systems::Wam<4>::gravityCompensate, Wam4_gravityCompensate_overloads())
    ;

    class_<systems::Wam<7>, boost::noncopyable>("Wam7", no_init)
    	.def("gravityCompensate", &systems::Wam<7>::gravityCompensate, Wam7_gravityCompensate_overloads())
    ;

    class_<ProductManager, boost::noncopyable>("ProductManager")
    	.def("getExecutionManager", &ProductManager::getExecutionManager, ProductManager_getExecutionManager_overloads()[return_internal_reference<>()])
    	.def("waitForWam", &ProductManager::waitForWam, ProductManager_waitForWam_overloads())
    	.def("wakeAllPucks", &ProductManager::wakeAllPucks)
    	.def("foundWam7", &ProductManager::foundWam7)
    	.def("getWam7", &ProductManager::getWam7, ProductManager_getWam7_overloads()[return_internal_reference<>()])
    	.def("foundWam4", &ProductManager::foundWam4)
    	.def("getWam4", &ProductManager::getWam4, ProductManager_getWam4_overloads()[return_internal_reference<>()])
    ;

    class_<bus::CANSocket, boost::noncopyable>("CANSocket", init<int>())
    	.def("send", &bus::CANSocket::send)
    	.def("receiveRaw", &bus::CANSocket::receiveRaw)
    ;

    class_<Puck>("Puck", init<bus::CANSocket&, int>())
		.def("wake", (void (Puck::*)())&Puck::wake)  // cast to resolve the overload
//    	.def("getProperty", &Puck::getProperty)
    ;

    def("takeAccelSample", &takeAccelSample);
    def("isWamActivated", &isWamActivated);
}

