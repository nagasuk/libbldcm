#include <libbldcm.hpp>
#include <libbldcm/register_map.hpp>

#include <memory>
#include <limits>
#include <utility>
#include <stdexcept>
#include <chrono>
#include <ratio>
#include <cmath>

using std::shared_ptr;
using std::numeric_limits;
using std::pair;
using std::make_pair;
using std::runtime_error;
using std::out_of_range;
using std::chrono::duration_cast;
using std::nano;
using std::chrono::nanoseconds;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::round;

namespace bldcm {

//========  Motor class ========
// Public
template<typename ClkFqType>
Motor::Motor(const shared_ptr<Fpgasoc> &ptr, const ClkFqType &clkFq, const uint32_t baseAddr)
	: _regmap(ptr, baseAddr), _clkFq(clockFreq_cast<Hz>(clkFq))
{
	// Try to fetch HW IP version and deadtime.
	this->_regmap.stat.updateCache();
	this->_fetchHwIpVersion(true);
	this->_fetchDeadtime(true);

	// Try to fetch PWM duty
	this->_calcPwmDutyFromRegister();
}

template Motor::Motor<Hz>(const shared_ptr<Fpgasoc>&, const Hz&, const uint32_t);
template Motor::Motor<KHz>(const shared_ptr<Fpgasoc>&, const KHz&, const uint32_t);
template Motor::Motor<MHz>(const shared_ptr<Fpgasoc>&, const MHz&, const uint32_t);

template<typename RotationalSpeedType> // RotationalSpeedType is Rps or Rpm.
void Motor::rotationalSpeed(const RotationalSpeedType &speed) noexcept(false)
{
	const uint32_t rps = static_cast<uint32_t>(rotationalSpeed_cast<Rps>(speed).count());
	this->_regmap.freqtgt.freqtgt(rps);
}

template void Motor::rotationalSpeed<Rpm>(const Rpm&) noexcept(false);
template void Motor::rotationalSpeed<Rps>(const Rps&) noexcept(false);

template<typename RotationalSpeedType>
RotationalSpeedType Motor::rotationalSpeed() noexcept(false)
{
	const uint32_t rpstmp = this->_regmap.freqtgt.freqtgt();
	const Rps rps(rpstmp);

	return rotationalSpeed_cast<RotationalSpeedType>(rps);
}

template Rps Motor::rotationalSpeed<Rps>() noexcept(false);
template Rpm Motor::rotationalSpeed<Rpm>() noexcept(false);

void Motor::pwmDuty(const int duty) noexcept(false)
{
	const CtrlReg::CacheState cacheStatus = this->_regmap.ctrl.cacheStatus();
	uint16_t pwmMaxcnt;
	uint32_t pwmCmp;

	if (cacheStatus == CtrlReg::CacheState::initialized) {
		pwmMaxcnt = this->_regmap.ctrl.pwmMaxcnt();
	} else if (cacheStatus == CtrlReg::CacheState::sync) {
		pwmMaxcnt = this->_regmap.ctrl.pwmMaxcnt(true);
	} else {
		throw runtime_error("Try to fetch pwmMaxcnt but the reg cache is modified.");
	}

	if (duty == static_cast<int>(100)) {
		pwmCmp = pwmMaxcnt + static_cast<int>(1); 
	} else if ((duty >= static_cast<int>(0)) && (duty < static_cast<int>(100))) {
		pwmCmp = (pwmMaxcnt * static_cast<uint32_t>(duty)) / static_cast<uint32_t>(100U);
	} else {
		throw out_of_range("PwmDuty is out of range.");
	}

	this->_regmap.pwmCmp.pwmCmp(pwmCmp);

	this->_pwmDuty = make_pair(true, duty);
}

int  Motor::pwmDuty() noexcept(false)
{
	// This const value can be updated, because it's reference value.
	const bool &isPwmDutyValid = this->_pwmDuty.first;

	if (!isPwmDutyValid) {
		this->_calcPwmDutyFromRegister();
	}

	if (!isPwmDutyValid) {
		throw runtime_error("PWM duty cannot be fetched from register.");
	}

	return this->_pwmDuty.second;
}

void Motor::outputEnable(bool isEnable) noexcept(false)
{
	if (this->_regmap.ctrl.cacheStatus() != CtrlReg::CacheState::modified) {
		const uint8_t writeVal = (isEnable) ? CtrlReg::En::Val::Enable : CtrlReg::En::Val::Disable;
		this->_regmap.ctrl.en(writeVal);
	} else {
		throw runtime_error("Cache of CtrlReg is modified at writing CTRL.EN.");
	}
}

bool Motor::outputEnable() noexcept(false)
{
	bool ret = false;

	if (this->_regmap.ctrl.cacheStatus() != CtrlReg::CacheState::modified) {
		const uint8_t readVal = this->_regmap.ctrl.en();

		if (readVal == CtrlReg::En::Val::Enable) {
			ret = true;
		} else if (readVal == CtrlReg::En::Val::Disable) {
			// Do nothing.
		} else {
			throw runtime_error("The value read from CTRL.EN is garbled.");
		}
	}

	return ret;
}

template<typename PeriodType>
void Motor::pwmPeriod(const PeriodType &period, const int prsc) noexcept(false)
{
	const nanoseconds periodNs = duration_cast<nanoseconds>(period);
	// periodMaxCountNs = (((_MaxPwmcnt * 2) * 2^prsc) / clockFreq) 10^9;
	const nanoseconds::rep periodMaxCountNs = ((_MaxPwmMaxcnt * nano::den) << (prsc + static_cast<int>(1))) / (nano::num * this->_clkFq.count());
	const nanoseconds periodMax(periodMaxCountNs);
	uint16_t pwmMaxcnt;

	if ((prsc > _MaxPrscSel) || (prsc < _MinPrscSel)) {
		throw out_of_range("Prescaler selection # is out of range.");
	}

	if (periodNs > periodMax) {
		throw out_of_range("Combination of period and prescaler is out of range.");
	}

	//pwmMaxcnt = ((period[ns] * clockFreq[Hz]) / (2^prsc * 2)) * 10^(-9);
	pwmMaxcnt = static_cast<uint16_t>(((periodNs.count() * this->_clkFq.count() * nano::num) / nano::den) >> (prsc + static_cast<int>(1)));

	if (this->_regmap.ctrl.cacheStatus() != CtrlReg::CacheState::modified) {
		this->_regmap.ctrl.pwmMaxcnt(pwmMaxcnt, true);
		this->_regmap.ctrl.pwmPrsc(static_cast<uint8_t>(prsc), true);
		this->_regmap.ctrl.flushCache();
		// Update PWM_CMP based on duty.
		this->pwmDuty(this->pwmDuty());
	} else {
		throw runtime_error("Cache of CtrlReg is modified at trying flushing PWM Period.");
	}
}

template void Motor::pwmPeriod<nanoseconds>(const nanoseconds &period, const int prsc) noexcept(false);
template void Motor::pwmPeriod<microseconds>(const microseconds &period, const int prsc) noexcept(false);
template void Motor::pwmPeriod<milliseconds>(const milliseconds &period, const int prsc) noexcept(false);
template void Motor::pwmPeriod<seconds>(const seconds &period, const int prsc) noexcept(false);

template<typename PeriodType>
pair<PeriodType, int> Motor::pwmPeriod() noexcept(false)
{
	uint8_t  pwmPrsc;
	uint16_t pwmMaxcnt;
	nanoseconds::rep countNs;

	if (this->_regmap.ctrl.cacheStatus() != CtrlReg::CacheState::modified) {
		this->_regmap.ctrl.updateCache();
		pwmPrsc = this->_regmap.ctrl.pwmPrsc(true);
		pwmMaxcnt = this->_regmap.ctrl.pwmMaxcnt(true);
	} else {
		throw runtime_error("Cache of CtrlReg is modified at trying fetching PWM Period.");
	}

	//countNs = (((pwmMaxcnt * 2) * 2^prsc) / clockFreq) * 10^9;
	countNs = ((pwmMaxcnt * nano::den) << (pwmPrsc + static_cast<uint8_t>(1))) / (nano::num * this->_clkFq.count());

	return make_pair(duration_cast<PeriodType>(nanoseconds(countNs)), static_cast<int>(pwmPrsc));
}

template pair<nanoseconds, int> Motor::pwmPeriod<nanoseconds>() noexcept(false);
template pair<microseconds, int> Motor::pwmPeriod<microseconds>() noexcept(false);
template pair<milliseconds, int> Motor::pwmPeriod<milliseconds>() noexcept(false);
template pair<seconds, int> Motor::pwmPeriod<seconds>() noexcept(false);

void Motor::phase(const int phase) noexcept(false)
{
	if ((phase < _MinPhase) || (phase > _MaxPhase)) {
		throw out_of_range("Phase is out of range.");
	}

	this->_regmap.ctrl.phase(static_cast<uint8_t>(phase));
}

int Motor::phase() noexcept(false)
{
	return static_cast<int>(this->_regmap.ctrl.phase());
}

const std::string &Motor::hwIpVersion() noexcept(false)
{
	// Check whether HW IP version is valid.
	if (!this->_hwIpVersion.first) {
		if (this->_regmap.stat.cacheStatus() == StatReg::CacheState::sync) {
			this->_fetchHwIpVersion(true);
		} else if (this->_regmap.stat.cacheStatus() == StatReg::CacheState::initialized) {
			this->_fetchHwIpVersion(false);
		} else {
			throw runtime_error("Cache is modified at trying fetching HW IP version.");
		}
	}

	if (!this->_hwIpVersion.first) {
		throw runtime_error("Fail to fetch HW IP version.");
	}

	return this->_hwIpVersion.second;
}

const int &Motor::deadtime()
{
	// Check whether HW IP version is valid.
	if (!this->_deadtime.first) {
		if (this->_regmap.stat.cacheStatus() == StatReg::CacheState::sync) {
			this->_fetchDeadtime(true);
		} else if (this->_regmap.stat.cacheStatus() == StatReg::CacheState::initialized) {
			this->_fetchDeadtime(false);
		} else {
			throw runtime_error("Cache is modified at trying fetching deadtime.");
		}
	}

	if (!this->_deadtime.first) {
		throw runtime_error("Fail to fetch deadtime.");
	}

	return this->_deadtime.second;
}

bool Motor::isReflectedFreq() noexcept(false)
{
	bool ret = false;

	if (this->_regmap.stat.cacheStatus() != StatReg::CacheState::modified) {
		if (this->_regmap.stat.reflectedfreq() == StatReg::Reflectedfreq::Val::Reflected) {
			ret = true;
		}
	} else {
		throw runtime_error("Cache is modified at trying fetching STAT.REFLECTEDFREQ flug.");
	}

	return ret;
}

bool Motor::isStopping() noexcept(false)
{
	bool ret = false;

	if (this->_regmap.stat.cacheStatus() != StatReg::CacheState::modified) {
		if (this->_regmap.stat.stop() == StatReg::Stop::Val::Stopping) {
			ret = true;
		}
	} else {
		throw runtime_error("Cache is modified at trying fetching STAT.STOP flug.");
	}

	return ret;
}

// Private
void Motor::_fetchHwIpVersion(const bool fromCache) noexcept(true)
{
	bool isFetchFail = false;
	uint8_t relCnt = numeric_limits<uint8_t>::max();

	try {
		relCnt = this->_regmap.stat.relCnt(fromCache);

	} catch (const std::range_error &e) {
		isFetchFail = true;
	}

	if (!isFetchFail) {
		if (relCnt <= StatReg::RelCnt::MaxVal) {
			this->_hwIpVersion = make_pair(true, StatReg::RelCnt::VerTbl[relCnt]);
		} 
	}
}

void Motor::_fetchDeadtime(const bool fromCache) noexcept(true)
{
	bool    isFetchFail = false;
	uint8_t deadtime = numeric_limits<uint8_t>::max();

	try {
		deadtime = this->_regmap.stat.deadtime(fromCache);

	} catch (const std::range_error &e) {
		isFetchFail = true;
	}

	if (!isFetchFail) {
		this->_deadtime = make_pair(true, static_cast<int>(deadtime));
	}
}

void Motor::_calcPwmDutyFromRegister() noexcept(true)
{
	const CtrlReg::CacheState   cacheStatusCtrl   = this->_regmap.ctrl.cacheStatus();
	const PwmCmpReg::CacheState cacheStatusPwmCmp = this->_regmap.pwmCmp.cacheStatus();
	uint16_t pwmMaxcnt;
	uint32_t pwmCmp;
	bool     isFetchFail = false;

	try {
		if ((cacheStatusCtrl   != CtrlReg::CacheState::modified) &&
		    (cacheStatusPwmCmp != PwmCmpReg::CacheState::modified)) {
			pwmCmp = this->_regmap.pwmCmp.pwmCmp();
			pwmMaxcnt = this->_regmap.ctrl.pwmMaxcnt();
		} else {
			throw runtime_error("Try to fetch pwmMaxcnt or pwmCmp but the reg cache is modified.");
		}
	} catch(...) {
		isFetchFail = true;
	}

	if (!isFetchFail) {
		if (pwmCmp > static_cast<uint32_t>(pwmMaxcnt)) {
			this->_pwmDuty = make_pair(true, static_cast<int>(100));
		} else if (pwmMaxcnt > static_cast<uint16_t>(0U)) {
			double dutyl = (static_cast<double>(pwmCmp) * static_cast<double>(100.)) / static_cast<double>(pwmMaxcnt);
			int duty = static_cast<int>(round(dutyl));
			this->_pwmDuty = make_pair(true, duty);
		} else {
			this->_pwmDuty = make_pair(false, _InvalidPwmDuty);
		}
	}
}

} // End of "namespace bldcm"

