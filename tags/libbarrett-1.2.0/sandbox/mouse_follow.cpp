/*
 * mouse_follow.cpp
 *
 *  Created on: May 24, 2011
 *      Author: dc
 */

#include <string>

#include <unistd.h> // For close()
#include <sys/socket.h> // For sockets
#include <fcntl.h>      // To change socket to nonblocking mode
#include <arpa/inet.h>  // For inet_pton()

#include <boost/tuple/tuple.hpp>

#include <barrett/units.h>
#include <barrett/os.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>

#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>


using namespace barrett;
using detail::waitForEnter;
BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

//template <typename T> rateLimit(const T& x, const T& x_1, const T& limit) {
//	if ( ((x - x_1) > limit).any() ) {
//		return
//	}
//}



class MouseFollowSystem : public systems::SingleIO<pose_type, pose_type> {
public:
	explicit MouseFollowSystem(systems::ExecutionManager* em, const std::string& sysName = "MouseFollowSystem") :
		systems::SingleIO<pose_type, pose_type>(sysName), numMissed(NUM_MISSED_LIMIT)
	{
		if (em != NULL) {
			// Make sure operate() gets called every execution cycle.
			em->startManaging(*this);
		}
	}
	virtual ~MouseFollowSystem() { mandatoryCleanUp(); }

	void command(double* inputs) {
		BARRETT_SCOPED_LOCK(getEmMutex());

		if (numMissed >= NUM_MISSED_LIMIT) {
			basePos = data.get<0>();
			baseQ = data.get<1>();
			comAA.axis() = baseQ.inverse() * Eigen::Vector3d::UnitZ();
			comAA.angle() = 0.0;

			comPos.setZero();
			inputPos_1.setZero();
		}

		inputPos[0] = inputs[0];
		inputPos[1] = inputs[1];
		inputPos[2] = inputs[2];

		comPos += data.get<1>().inverse() * (inputPos - inputPos_1);
		inputPos_1 = inputPos;

		comAngle = inputs[3];

		numMissed = 0;
	}

protected:
	static const int NUM_MISSED_LIMIT = 50;
	static const cp_type CP_RATE_LIMIT;
	static const double ANGLE_RATE_LIMIT = 0.001;

	int numMissed;
	cp_type basePos, comPos, inputPos, inputPos_1;
	Eigen::Quaterniond baseQ;
	Eigen::AngleAxisd comAA;
	double comAngle;
	pose_type data;

	cp_type delta;

	virtual void operate() {
		if (numMissed < NUM_MISSED_LIMIT) {  // prevent numMissed from wrapping
			++numMissed;
		}

		if (numMissed >= NUM_MISSED_LIMIT) {
			data = input.getValue();
		} else {
			delta = basePos + comPos - data.get<0>();
			data.get<0>() += math::sign(delta).cwise() * math::min(math::abs(delta), CP_RATE_LIMIT);

			double angleDiff = comAngle - comAA.angle();
			if (math::abs(angleDiff) > ANGLE_RATE_LIMIT) {
				comAA.angle() += math::sign(angleDiff) * ANGLE_RATE_LIMIT;
			} else {
				comAA.angle() = comAngle;
			}
			data.get<1>() = baseQ * comAA;

		}

		outputValue->setData(&data);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(MouseFollowSystem);
};
const cp_type MouseFollowSystem::CP_RATE_LIMIT(0.0002);


bool validate_args(int argc, char** argv) {
	if (argc != 2) {
		printf("Usage: %s <remoteHost>\n", argv[0]);
		return false;
	}
	return true;
}

int openSocket(const char* remoteHost, int port = 5555);


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wam.gravityCompensate();

	Hand* hand = NULL;
	int ot = 0, ct = 0;
	if (pm.foundHand()) {
		hand = pm.getHand();

		printf("Press [Enter] to initialize Hand. (Make sure it has room!)");
		waitForEnter();
		hand->initialize();

		ot = hand->getPucks()[3]->getProperty(Puck::OT);
		ct = hand->getPucks()[3]->getProperty(Puck::CT);
	}

	MouseFollowSystem mfs(pm.getExecutionManager());
	systems::connect(wam.toolPose.output, mfs.input);
	wam.trackReferenceSignal(mfs.output);


	const size_t NUM_DOUBLES = 6;
	const int SIZE_OF_MSG = NUM_DOUBLES*sizeof(double);

	int sock = openSocket(argv[1]);
	math::Vector<NUM_DOUBLES>::type inputs;
	Hand::jv_type hjv(0.0);
	double fv_1 = 0.0, sv_1 = 0.0;
	while (true) {
		bool received = false;
		while (recv(sock, inputs.data(), SIZE_OF_MSG, 0) == SIZE_OF_MSG) {
			received = true;
		}

		if (received) {
//			std::cout << inputs << "\n";
			mfs.command(inputs.data());

			if (hand != NULL) {
				if (fv_1 != inputs[4]  ||  sv_1 != inputs[5]) {
					hjv.setConstant(inputs[4]);
					hjv[3] = inputs[5];

					hand->update(Hand::S_POSITION);
					int s = hand->getPrimaryEncoderPosition()[3];
					if (ct - s > 5000  &&  s - ot > 5000) {
						hjv[2] = 0.0;
					}

					hand->velocityMove(hjv);

					fv_1 = inputs[4];
					sv_1 = inputs[5];
				}
			}
		}
		usleep(10000);
	}

/*
	Eigen::Quaterniond q;
	systems::ExposedOutput<Eigen::Quaterniond> toSetPoint;

	printf("Press [Enter] to hold orientation.");
	waitForEnter();
	q = wam.getToolOrientation();
	toSetPoint.setValue(q);
	wam.trackReferenceSignal(toSetPoint.output);


	while (true) {
		usleep(10000);
		Eigen::Vector3d axis = q.inverse() * Eigen::Vector3d::UnitZ();
		Eigen::AngleAxisd aa(0.0003, axis);
		q = q * aa;
		toSetPoint.setValue(q);
//		printf("%f,%f,%f,%f\n", q.x(), q.y(), q.z(), q.w());
	}
*/

	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}

int openSocket(const char* remoteHost, int port) {
	int sock;

	int err;
	long flags;
	struct sockaddr_in bind_addr;
	struct sockaddr_in their_addr;

	/* Create socket */
	sock = socket(PF_INET, SOCK_DGRAM, 0);
	if (sock == -1)
	{
		(logMessage("%s: Could not create socket %d.") % __func__ % ret).raise<std::runtime_error>();
	}

	/* Set socket to non-blocking, set flag associated with open file */
	flags = fcntl(sock, F_GETFL, 0);
	if (flags < 0)
	{
		(logMessage("%s: Could not get socket flags %d.") % __func__ % ret).raise<std::runtime_error>();
	}
	flags |= O_NONBLOCK;
	err = fcntl(sock, F_SETFL, flags);
	if (err < 0)
	{
		syslog(LOG_ERR,"%s: Could not set socket flags.",__func__);
		throw std::runtime_error("openSocket(): Failed.");
	}

	/* Set up the bind address */
	bind_addr.sin_family = AF_INET;
	bind_addr.sin_port = htons(port);
	bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	err = bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
	if (err == -1)
	{
		syslog(LOG_ERR,"%s: Could not bind to socket on port %d.",__func__,port);
		throw std::runtime_error("openSocket(): Failed.");
	}

	/* Set up the other guy's address */
	their_addr.sin_family = AF_INET;
	their_addr.sin_port = htons(port);
	err = ! inet_pton(AF_INET, remoteHost, &their_addr.sin_addr);
	if (err)
	{
		syslog(LOG_ERR,"%s: Bad IP argument '%s'.",__func__,remoteHost);
		throw std::runtime_error("openSocket(): Failed.");
	}

	/* Call "connect" to set datagram destination */
	err = connect(sock, (struct sockaddr *)&their_addr, sizeof(struct sockaddr));
	if (err)
	{
		syslog(LOG_ERR,"%s: Could not set datagram destination.",__func__);
		throw std::runtime_error("openSocket(): Failed.");
	}

	return sock;
}

