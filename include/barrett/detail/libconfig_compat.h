/*
 * libconfig_compat.h
 *
 * Barrett's patched libconfig added Setting::getCSetting() so that settings
 * parsed with the C++ API could be passed to the cdlbt C code. Stock
 * libconfig (as shipped by Ubuntu) keeps the underlying config_setting_t*
 * private, so this header recovers it instead. The explicit instantiation of
 * SettingPtrThief below is allowed to name the private Setting::_setting
 * member ([temp.explicit]), making this standard-conforming.
 */

#ifndef BARRETT_DETAIL_LIBCONFIG_COMPAT_H_
#define BARRETT_DETAIL_LIBCONFIG_COMPAT_H_


#include <libconfig.h++>


namespace barrett {
namespace detail {


struct SettingPtrTag {
	typedef config_setting_t* libconfig::Setting::* MemberPtr;
	friend MemberPtr stealSettingPtr(SettingPtrTag);
};

template<SettingPtrTag::MemberPtr M>
struct SettingPtrThief {
	friend SettingPtrTag::MemberPtr stealSettingPtr(SettingPtrTag) { return M; }
};
template struct SettingPtrThief<&libconfig::Setting::_setting>;

inline config_setting_t* getCSetting(const libconfig::Setting& setting)
{
	return setting.*stealSettingPtr(SettingPtrTag());
}


}
}


#endif /* BARRETT_DETAIL_LIBCONFIG_COMPAT_H_ */
