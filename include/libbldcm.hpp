#ifndef LIBBLDCM_HPP
#define LIBBLDCM_HPP

#include <libbldcm/register_map.hpp>

#include <libfpgasoc.hpp>
#include <memory>
#include <utility>
#include <chrono>
#include <ratio>

#ifndef __cplusplus
#error "This library is one for C++."
#endif

#if __cplusplus < 201703L
#error "This library requres C++17 or higher."
#endif

namespace bldcm {

// Types
using Rps = std::chrono::duration< int64_t, std::ratio<1> >;
using Rpm = std::chrono::duration< int64_t, std::ratio<1, 60> >;

template<typename RSConverted, typename RSBase>
constexpr RSConverted rotationalSpeed_cast(const RSBase &rs)
{
	return std::chrono::duration_cast<RSConverted>(rs);
}

using Hz  = std::chrono::duration< int64_t, std::ratio<1> >;
using KHz = std::chrono::duration< int64_t, std::ratio<1000> >;
using MHz = std::chrono::duration< int64_t, std::ratio<1000000> >;

template<typename ClkFqConverted, typename ClkFqBase>
constexpr ClkFqConverted clockFreq_cast(const ClkFqBase &clkFq)
{
	return std::chrono::duration_cast<ClkFqConverted>(clkFq);
}

class Motor {
	public:
		// Constructor/destructor
		template<typename ClkFqType>
		Motor(const std::shared_ptr<Fpgasoc> &ptr, const ClkFqType &clkFq, const uint32_t baseAddr);
		~Motor() {}

		// Methods
		template<typename RotationalSpeedType> // RotationalSpeedType is Rps or Rpm.
		void rotationalSpeed(const RotationalSpeedType &speed) noexcept(false);
		template<typename RotationalSpeedType>
		RotationalSpeedType rotationalSpeed() noexcept(false);

		void pwmDuty(const int duty) noexcept(false);
		int  pwmDuty() noexcept(false);

		void outputEnable(bool isEnable) noexcept(false);
		bool outputEnable() noexcept(false);

		template<typename PeriodType> // PeriodType is nanoseconds, microseconds, milliseconds, or seconds.
		void pwmPeriod(const PeriodType &period, const int prsc) noexcept(false);
		template<typename PeriodType>
		std::pair<PeriodType, int> pwmPeriod() noexcept(false);

		void phase(const int phase) noexcept(false);
		int phase() noexcept(false);

		const std::string &hwIpVersion() noexcept(false);
		const int         &deadtime() noexcept(false);

		bool isReflectedFreq() noexcept(false);
		bool isStopping() noexcept(false);

	private:
		// Materials
		static constexpr char _InvalidHwIpVerStr[] = "UNKNOWN";
		static constexpr int  _InvalidDeadtime     = static_cast<int>(-1);
		static constexpr int  _InvalidPwmDuty      = static_cast<int>(-1);

		static constexpr int _MinPrscSel = 0;
		static constexpr int _MaxPrscSel = 32;

		static constexpr uint16_t _MaxPwmMaxcnt = static_cast<uint16_t>(0xFFFFU);

		static constexpr int _MinPhase = static_cast<int>(0);
		static constexpr int _MaxPhase = static_cast<int>(5);

		// Members
		RegMap _regmap;
		const Hz _clkFq;
		std::pair<bool, std::string> _hwIpVersion = std::make_pair(false, _InvalidHwIpVerStr);
		std::pair<bool, int>         _deadtime    = std::make_pair(false, _InvalidDeadtime);
		std::pair<bool, int>         _pwmDuty     = std::make_pair(false, _InvalidPwmDuty);

		// Methods
		void _fetchHwIpVersion(const bool fromCache) noexcept(true);
		void _fetchDeadtime(const bool fromCache) noexcept(true);
		void _calcPwmDutyFromRegister() noexcept(true);
};

} // End of "namespace bldcm"

#endif // End of "#ifndef LIBBLDCM_HPP"

