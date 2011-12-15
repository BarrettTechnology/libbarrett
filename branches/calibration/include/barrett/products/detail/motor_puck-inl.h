/*
 * motor_puck-inl.h
 *
 *  Created on: Feb 1, 2011
 *      Author: dc
 */

namespace barrett {


template<typename ResultType>
int MotorPuck::MotorPositionParser<ResultType>::parse(int id, int propId, result_type* result, const unsigned char* data, size_t len) {
	if (len != 3 && len != 6) {
		syslog(LOG_ERR,
				"%s: expected message length of 3 or 6, got message length of %d",
				__func__, len);
		return 1;
	}

	*result = twentyTwoBit2<ResultType>(data[0], data[1], data[2]);
	return 0;
}
template<typename ResultType>
int MotorPuck::SecondaryPositionParser<ResultType>::parse(int id, int propId, result_type* result, const unsigned char* data, size_t len) {
	if (len != 3) {
		syslog(LOG_ERR,
				"%s: expected message length of 3, got message length of %d",
				__func__, len);
		return 1;
	}

	*result = twentyTwoBit2<ResultType>(data[0], data[1], data[2]);
	return 0;
}
template<typename ResultType>
int MotorPuck::CombinedPositionParser<ResultType>::parse(int id, int propId, result_type* result, const unsigned char* data, size_t len) {
	if (len == 6) {
		boost::get<0>(*result) = twentyTwoBit2<ResultType>(data[0], data[1], data[2]);
		boost::get<1>(*result) = twentyTwoBit2<ResultType>(data[3], data[4], data[5]);
	} else if (len == 3) {
		boost::get<0>(*result) = twentyTwoBit2<ResultType>(data[0], data[1], data[2]);
		boost::get<1>(*result) = std::numeric_limits<ResultType>::max();
	} else {
		syslog(LOG_ERR,
				"%s: expected message length of 3 or 6, got message length of %d",
				__func__, len);
		return 1;
	}

	return 0;
}


template<typename ResultType>
ResultType MotorPuck::twentyTwoBit2(unsigned char msb, unsigned char middle, unsigned char lsb)
{
	int intResult = 0;
	intResult |= ((long) msb << 16) & 0x003F0000;
	intResult |= ((long) middle << 8) & 0x0000FF00;
	intResult |= ((long) lsb) & 0x000000FF;

	if (intResult & 0x00200000) {  // If negative...
		intResult |= ~((int)0x3fffff); // sign-extend
	}

	return intResult;
}


}
